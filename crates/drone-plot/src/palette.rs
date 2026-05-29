use plotters::style::RGBColor;

/// Paleta 6 kontrastowych kolorów (kolejność jak w matplotlib tab10).
pub const PALETTE: [RGBColor; 6] = [
    RGBColor(0x1f, 0x77, 0xb4), // niebieski
    RGBColor(0xff, 0x7f, 0x0e), // pomarańczowy
    RGBColor(0x2c, 0xa0, 0x2c), // zielony
    RGBColor(0xd6, 0x27, 0x28), // czerwony
    RGBColor(0x94, 0x67, 0xbd), // fioletowy
    RGBColor(0x8c, 0x56, 0x4b), // brązowy
];
