use std::fs;
use std::path::Path;
use std::process::{Command, Output};

use manifold_csg::Manifold;
use tempfile::tempdir;
use threemf::model::{Mesh, Model, Unit};

fn cli(dir: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cadhr-lang"))
        .current_dir(dir)
        .args(args)
        .output()
        .unwrap()
}

fn assert_success(result: &Output) {
    assert!(
        result.status.success(),
        "stdout: {}\nstderr: {}",
        String::from_utf8_lossy(&result.stdout),
        String::from_utf8_lossy(&result.stderr),
    );
}

fn read_model(path: &Path) -> Model {
    let mut models = threemf::read(fs::File::open(path).unwrap()).unwrap();
    assert_eq!(models.len(), 1);
    let model = models.pop().unwrap();
    assert!(matches!(model.unit, Unit::Millimeter));
    assert_eq!(model.resources.object.len(), model.build.item.len());
    for (object, item) in model.resources.object.iter().zip(&model.build.item) {
        assert_eq!(object.id, item.objectid);
        assert!(item.transform.is_none());
    }
    model
}

fn mesh_volume(mesh: &Mesh) -> f64 {
    let vertices: Vec<f64> = mesh
        .vertices
        .vertex
        .iter()
        .flat_map(|v| [v.x, v.y, v.z])
        .collect();
    let indices: Vec<u64> = mesh
        .triangles
        .triangle
        .iter()
        .flat_map(|t| [t.v1 as u64, t.v2 as u64, t.v3 as u64])
        .collect();
    Manifold::from_mesh_f64(&vertices, 3, &indices)
        .unwrap()
        .volume()
}

#[test]
fn exports_main_as_a_closed_mesh_in_millimeters() {
    let dir = tempdir().unwrap();
    fs::write(
        dir.path().join("db.cadhr"),
        "main = { models = [cube 10.0 20.0 30.0], bom = [], controls = [control3d \"origin\" (p3 0.0 0.0 0.0)] }",
    )
    .unwrap();

    assert_success(&cli(dir.path(), &["export", "db.cadhr", "-o", "model.3mf"]));

    let model = read_model(&dir.path().join("model.3mf"));
    assert_eq!(model.resources.object.len(), 1);
    let mesh = model.resources.object[0].mesh.as_ref().unwrap();
    assert_eq!(mesh.vertices.vertex.len(), 8);
    assert_eq!(mesh.triangles.triangle.len(), 12);
    approx::assert_abs_diff_eq!(mesh_volume(mesh), 6000.0, epsilon = 1e-8);
}

#[test]
fn exports_a_named_binding_from_a_project_directory_with_multiple_models() {
    let dir = tempdir().unwrap();
    fs::create_dir(dir.path().join("Project")).unwrap();
    fs::write(
        dir.path().join("Project/db.cadhr"),
        "main = cube 1.0 1.0 1.0\nparts = [cube 2.0 3.0 4.0, cube 5.0 6.0 7.0 |> translate3d (p3 0.0 0.0 0.0) (p3 30.0 0.0 0.0)]",
    )
    .unwrap();

    assert_success(&cli(
        dir.path(),
        &[
            "export",
            "Project",
            "--binding",
            "parts",
            "--output",
            "parts.3mf",
        ],
    ));

    let model = read_model(&dir.path().join("parts.3mf"));
    assert_eq!(model.resources.object.len(), 2);
    for (index, volume) in [24.0, 210.0].into_iter().enumerate() {
        let object = &model.resources.object[index];
        assert_eq!(object.id, index + 1);
        approx::assert_abs_diff_eq!(
            mesh_volume(object.mesh.as_ref().unwrap()),
            volume,
            epsilon = 1e-8
        );
    }
    let mesh = model.resources.object[1].mesh.as_ref().unwrap();
    assert!(
        mesh.vertices
            .vertex
            .iter()
            .all(|v| (30.0..=35.0).contains(&v.x))
    );
}

#[test]
fn uses_slider_defaults_for_binding_parameters() {
    let dir = tempdir().unwrap();
    fs::write(
        dir.path().join("db.cadhr"),
        "part size = cube size 2.0 3.0\nslider part.size = 2.0 .. 6.0",
    )
    .unwrap();

    assert_success(&cli(
        dir.path(),
        &["export", ".", "-b", "part", "-o", "part.3mf"],
    ));

    let model = read_model(&dir.path().join("part.3mf"));
    approx::assert_abs_diff_eq!(
        mesh_volume(model.resources.object[0].mesh.as_ref().unwrap()),
        24.0,
        epsilon = 1e-8,
    );
}

#[test]
fn resolves_sibling_modules_and_bundled_std_and_skips_empty_models() {
    let dir = tempdir().unwrap();
    fs::create_dir(dir.path().join("Parts")).unwrap();
    fs::create_dir(dir.path().join("Project")).unwrap();
    fs::write(
        dir.path().join("Parts/db.cadhr"),
        "module Parts exposing (body)\nbody = cube 2.0 3.0 4.0",
    )
    .unwrap();
    fs::write(
        dir.path().join("Project/db.cadhr"),
        "import Parts exposing (body)\nimport Std\nempty = diff3d body body\nmain = [empty, body, empty]",
    )
    .unwrap();

    assert_success(&cli(
        dir.path(),
        &["export", "Project/db.cadhr", "-o", "model.3mf"],
    ));

    let model = read_model(&dir.path().join("model.3mf"));
    assert_eq!(model.resources.object.len(), 1);
    assert_eq!(model.resources.object[0].id, 1);
    approx::assert_abs_diff_eq!(
        mesh_volume(model.resources.object[0].mesh.as_ref().unwrap()),
        24.0,
        epsilon = 1e-8,
    );
}

#[test]
fn preserves_double_precision_coordinates() {
    let dir = tempdir().unwrap();
    fs::write(
        dir.path().join("db.cadhr"),
        "main = cube 1.0 1.0 1.0 |> translate3d (p3 0.0 0.0 0.0) (p3 100000000.125 0.0 0.0)",
    )
    .unwrap();

    assert_success(&cli(dir.path(), &["export", ".", "-o", "model.3mf"]));

    let model = read_model(&dir.path().join("model.3mf"));
    let mesh = model.resources.object[0].mesh.as_ref().unwrap();
    let min_x = mesh
        .vertices
        .vertex
        .iter()
        .map(|v| v.x)
        .fold(f64::INFINITY, f64::min);
    let max_x = mesh
        .vertices
        .vertex
        .iter()
        .map(|v| v.x)
        .fold(f64::NEG_INFINITY, f64::max);
    assert_eq!(min_x, 100000000.125);
    assert_eq!(max_x, 100000001.125);
    approx::assert_abs_diff_eq!(mesh_volume(mesh), 1.0, epsilon = 1e-8);
}

#[test]
fn failed_exports_do_not_create_or_overwrite_output() {
    for (source, error) in [
        ("main = undefined_name", "undefined_name"),
        ("part = cube 1.0 1.0 1.0", "main"),
        ("main = 42", "run main"),
        ("main = []", "nothing to export"),
        (
            "main = diff3d (cube 1.0 1.0 1.0) (cube 1.0 1.0 1.0)",
            "nothing to export",
        ),
        ("main = sweep_extrude_xy [] (circle 1.0)", "mesh (model #1)"),
        (
            "main = [cube 1.0 1.0 1.0, sweep_extrude_xy [] (circle 1.0)]",
            "mesh (model #2)",
        ),
    ] {
        let dir = tempdir().unwrap();
        fs::write(dir.path().join("db.cadhr"), source).unwrap();
        for existing_output in [false, true] {
            if existing_output {
                fs::write(dir.path().join("model.3mf"), b"existing output").unwrap();
            }
            let result = cli(dir.path(), &["export", ".", "-o", "model.3mf"]);
            let stderr = String::from_utf8_lossy(&result.stderr);
            assert_eq!(result.status.code(), Some(1), "{source}: {stderr}");
            assert!(stderr.contains(error), "{source}: {stderr}");
            if existing_output {
                assert_eq!(
                    fs::read(dir.path().join("model.3mf")).unwrap(),
                    b"existing output"
                );
            } else {
                assert!(!dir.path().join("model.3mf").exists());
            }
        }
    }
}

#[test]
fn missing_input_returns_an_error_without_creating_output() {
    let dir = tempdir().unwrap();
    for path in ["missing.cadhr", "."] {
        let result = cli(dir.path(), &["export", path, "-o", "model.3mf"]);
        assert_eq!(result.status.code(), Some(2));
        assert!(String::from_utf8_lossy(&result.stderr).contains("not found"));
        assert!(!dir.path().join("model.3mf").exists());
    }
}

#[test]
fn reports_output_write_errors() {
    let dir = tempdir().unwrap();
    fs::write(dir.path().join("db.cadhr"), "main = cube 1.0 1.0 1.0").unwrap();

    for path in ["missing/model.3mf", "."] {
        let result = cli(dir.path(), &["export", ".", "-o", path]);
        assert_eq!(result.status.code(), Some(1));
        assert!(String::from_utf8_lossy(&result.stderr).contains(&format!("write {path}:")));
    }
}

#[test]
fn output_path_is_required() {
    let dir = tempdir().unwrap();
    let result = cli(dir.path(), &["export", "."]);
    assert_eq!(result.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&result.stderr).contains("--output"));
}

#[test]
fn check_still_supports_library_and_binding_checks() {
    let dir = tempdir().unwrap();
    fs::write(dir.path().join("db.cadhr"), "part = cube 1.0 1.0 1.0").unwrap();

    let result = cli(dir.path(), &["check", "."]);
    assert_success(&result);
    assert!(String::from_utf8_lossy(&result.stdout).contains("[lib]"));

    let result = cli(dir.path(), &["check", ".", "-b", "part"]);
    assert_success(&result);
    assert!(String::from_utf8_lossy(&result.stdout).contains("part: models=1, tris=12"));
}
