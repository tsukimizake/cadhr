use iced::keyboard;
use iced::widget::button::{self, Status};
use iced::widget::text_editor::{Binding, KeyPress};
use iced::{Background, Border, Color, Shadow};

pub fn dark_button_style(_theme: &iced::Theme, status: Status) -> button::Style {
    let bg = Color::from_rgb(0.24, 0.24, 0.26);
    let bg_hover = Color::from_rgb(0.34, 0.34, 0.36);
    let bg_pressed = Color::from_rgb(0.18, 0.18, 0.20);
    let text = Color::from_rgb(0.88, 0.88, 0.90);

    match status {
        Status::Active => button::Style {
            background: Some(Background::Color(bg)),
            text_color: text,
            border: Border::default().rounded(4),
            shadow: Shadow::default(),
            snap: false,
        },
        Status::Hovered => button::Style {
            background: Some(Background::Color(bg_hover)),
            text_color: text,
            border: Border::default().rounded(4),
            shadow: Shadow::default(),
            snap: false,
        },
        Status::Pressed => button::Style {
            background: Some(Background::Color(bg_pressed)),
            text_color: text,
            border: Border::default().rounded(4),
            shadow: Shadow::default(),
            snap: false,
        },
        Status::Disabled => button::Style {
            background: Some(Background::Color(Color::from_rgb(0.18, 0.18, 0.20))),
            text_color: Color::from_rgb(0.45, 0.45, 0.47),
            border: Border::default().rounded(4),
            shadow: Shadow::default(),
            snap: false,
        },
    }
}

pub fn dark_button<'a, Msg: Clone + 'a>(label: &'a str) -> iced::widget::Button<'a, Msg> {
    iced::widget::button(label).style(dark_button_style)
}

// TODO undo/redo
pub fn emacs_key_binding<Msg>(key_press: KeyPress) -> Option<Binding<Msg>> {
    let KeyPress {
        key,
        modifiers,
        status,
        ..
    } = &key_press;

    if !matches!(status, iced::widget::text_editor::Status::Focused { .. }) {
        return None;
    }

    if !modifiers.control() || modifiers.shift() || modifiers.alt() || modifiers.command() {
        return Binding::from_key_press(key_press);
    }

    match key.as_ref() {
        keyboard::Key::Character("a") => {
            Some(Binding::Move(iced::widget::text_editor::Motion::Home))
        }
        keyboard::Key::Character("e") => {
            Some(Binding::Move(iced::widget::text_editor::Motion::End))
        }
        keyboard::Key::Character("f") => {
            Some(Binding::Move(iced::widget::text_editor::Motion::Right))
        }
        keyboard::Key::Character("b") => {
            Some(Binding::Move(iced::widget::text_editor::Motion::Left))
        }
        keyboard::Key::Character("n") => {
            Some(Binding::Move(iced::widget::text_editor::Motion::Down))
        }
        keyboard::Key::Character("p") => Some(Binding::Move(iced::widget::text_editor::Motion::Up)),
        keyboard::Key::Character("d") => Some(Binding::Delete),
        keyboard::Key::Character("h") => Some(Binding::Backspace),
        keyboard::Key::Character("k") => Some(Binding::Sequence(vec![
            Binding::Select(iced::widget::text_editor::Motion::End),
            Binding::Copy,
            Binding::Delete,
        ])),
        keyboard::Key::Character("y") => Some(Binding::Paste),
        keyboard::Key::Character("w") => Some(Binding::Copy),
        keyboard::Key::Character("u") => Some(Binding::Sequence(vec![
            Binding::Select(iced::widget::text_editor::Motion::Home),
            Binding::Copy,
            Binding::Delete,
        ])),
        _ => Binding::from_key_press(key_press),
    }
}
