use bytemuck::{Pod, Zeroable};
use iced::advanced::graphics::Viewport;
use iced::wgpu;
use iced::wgpu::util::DeviceExt;
use iced::widget::shader;
use iced::Rectangle;
use std::collections::HashMap;
use std::sync::Arc;

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable, Debug)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    /// Per-vertex color. alpha=0 means "use uniform color" (main mesh).
    pub color: [f32; 4],
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable, Debug)]
pub struct Uniforms {
    pub view_proj: [[f32; 4]; 4],
    pub color: [f32; 4],
    pub light_dir: [f32; 4],
    pub edge_color: [f32; 4],
}

#[derive(Debug)]
pub struct MeshData {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    /// Sharp edge を強調表示するためのライン用インデックス (LineList)
    pub edge_indices: Vec<u32>,
}

struct PerInstance {
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    vertex_buffer: Option<wgpu::Buffer>,
    index_buffer: Option<wgpu::Buffer>,
    index_count: u32,
    edge_index_buffer: Option<wgpu::Buffer>,
    edge_index_count: u32,
    uploaded_version: u64,
}

pub struct Pipeline {
    pipeline: wgpu::RenderPipeline,
    edge_pipeline: wgpu::RenderPipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    instances: HashMap<u64, PerInstance>,
    depth_view: Option<wgpu::TextureView>,
    depth_size: (u32, u32),
}

impl shader::Pipeline for Pipeline {
    fn new(device: &wgpu::Device, _queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        Self::build(device, format)
    }
}

impl Pipeline {
    fn build(device: &wgpu::Device, format: wgpu::TextureFormat) -> Self {
        let shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cadhr_shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("cadhr_bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("cadhr_pl"),
            bind_group_layouts: &[&bgl],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cadhr_pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader_module,
                entry_point: Some("vs_main"),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x4],
                }],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader_module,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                cull_mode: Some(wgpu::Face::Back),
                front_face: wgpu::FrontFace::Ccw,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: Default::default(),
                // エッジを surface 表面に重ねるため、面側を僅かに奥へオフセット
                bias: wgpu::DepthBiasState {
                    constant: 1,
                    slope_scale: 1.0,
                    clamp: 0.0,
                },
            }),
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });

        let edge_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cadhr_edge_pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader_module,
                entry_point: Some("vs_edge"),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x4],
                }],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader_module,
                entry_point: Some("fs_edge"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });

        Self {
            pipeline,
            edge_pipeline,
            bind_group_layout: bgl,
            instances: HashMap::new(),
            depth_view: None,
            depth_size: (0, 0),
        }
    }

    pub fn update_instance(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        viewport: &Viewport,
        id: u64,
        uniforms: &Uniforms,
        mesh: &Arc<MeshData>,
        mesh_version: u64,
    ) {
        let size = viewport.physical_size();
        let target_size = (size.width.max(1), size.height.max(1));
        if self.depth_size != target_size {
            let depth = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("depth"),
                size: wgpu::Extent3d {
                    width: target_size.0,
                    height: target_size.1,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Depth32Float,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            });
            self.depth_view = Some(depth.create_view(&Default::default()));
            self.depth_size = target_size;
        }

        let bgl = &self.bind_group_layout;
        let inst = self.instances.entry(id).or_insert_with(|| {
            let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("uniforms"),
                size: std::mem::size_of::<Uniforms>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("bg"),
                layout: bgl,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: uniform_buffer.as_entire_binding(),
                }],
            });
            PerInstance {
                uniform_buffer,
                bind_group,
                vertex_buffer: None,
                index_buffer: None,
                index_count: 0,
                edge_index_buffer: None,
                edge_index_count: 0,
                uploaded_version: 0,
            }
        });

        queue.write_buffer(&inst.uniform_buffer, 0, bytemuck::bytes_of(uniforms));

        if mesh_version != 0 && mesh_version != inst.uploaded_version {
            // 空メッシュ (shape が 0 件) のときは buffer を作らずクリアする。
            // wgpu の create_buffer_init は空 slice で panic するため。
            if mesh.vertices.is_empty() || mesh.indices.is_empty() {
                inst.vertex_buffer = None;
                inst.index_buffer = None;
                inst.index_count = 0;
                inst.edge_index_buffer = None;
                inst.edge_index_count = 0;
                inst.uploaded_version = mesh_version;
                return;
            }
            inst.vertex_buffer = Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("vbuf"),
                contents: bytemuck::cast_slice(&mesh.vertices),
                usage: wgpu::BufferUsages::VERTEX,
            }));
            inst.index_buffer = Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("ibuf"),
                contents: bytemuck::cast_slice(&mesh.indices),
                usage: wgpu::BufferUsages::INDEX,
            }));
            inst.index_count = mesh.indices.len() as u32;
            if mesh.edge_indices.is_empty() {
                inst.edge_index_buffer = None;
                inst.edge_index_count = 0;
            } else {
                inst.edge_index_buffer =
                    Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("ebuf"),
                        contents: bytemuck::cast_slice(&mesh.edge_indices),
                        usage: wgpu::BufferUsages::INDEX,
                    }));
                inst.edge_index_count = mesh.edge_indices.len() as u32;
            }
            inst.uploaded_version = mesh_version;
        }
    }

    pub fn render_instance(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        clip_bounds: &Rectangle<u32>,
        id: u64,
    ) {
        let Some(depth_view) = self.depth_view.as_ref() else {
            return;
        };
        let Some(inst) = self.instances.get(&id) else {
            return;
        };
        let Some(vbuf) = inst.vertex_buffer.as_ref() else {
            return;
        };
        let Some(ibuf) = inst.index_buffer.as_ref() else {
            return;
        };

        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("cadhr_pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target,
                resolve_target: None,
                depth_slice: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });

        pass.set_viewport(
            clip_bounds.x as f32,
            clip_bounds.y as f32,
            clip_bounds.width as f32,
            clip_bounds.height as f32,
            0.0,
            1.0,
        );
        pass.set_scissor_rect(
            clip_bounds.x,
            clip_bounds.y,
            clip_bounds.width,
            clip_bounds.height,
        );
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &inst.bind_group, &[]);
        pass.set_vertex_buffer(0, vbuf.slice(..));
        pass.set_index_buffer(ibuf.slice(..), wgpu::IndexFormat::Uint32);
        pass.draw_indexed(0..inst.index_count, 0, 0..1);

        if let Some(ebuf) = inst.edge_index_buffer.as_ref() {
            if inst.edge_index_count > 0 {
                pass.set_pipeline(&self.edge_pipeline);
                pass.set_index_buffer(ebuf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..inst.edge_index_count, 0, 0..1);
            }
        }
    }

    pub fn remove_instance(&mut self, id: u64) {
        self.instances.remove(&id);
    }
}
