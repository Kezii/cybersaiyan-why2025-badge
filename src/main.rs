use embedded_gfx::{
    draw::draw,
    framebuffer::DmaReadyFramebuffer,
    mesh::{K3dMesh, RenderMode},
    perfcounter::PerformanceCounter,
    K3dengine,
};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::{IntoStorage, Point, WebColors},
    text::Text,
};
use esp_idf_svc::hal::peripheral::Peripheral;
use log::info;
use std::{f32::consts::PI, ffi::c_void};

use esp_idf_hal::{
    delay::Ets,
    gpio::{InputPin, Output, OutputPin, PinDriver},
    io::Write,
    ledc::{LedcChannel, LedcTimer},
    peripherals,
    prelude::*,
    spi::{Dma, SpiAnyPins, SpiDeviceDriver, SpiDriver, SpiDriverConfig},
};

use embedded_graphics::Drawable;

mod display_driver;

use display_interface_spi::SPIInterface;
use embedded_gfx::mesh::Geometry;
use load_stl::embed_stl;
use nalgebra::{Point2, Point3};

use crate::display_driver::FramebufferTarget;

#[allow(clippy::approx_constant)]
fn main() {
    esp_idf_svc::sys::link_patches();

    let peripherals = peripherals::Peripherals::take().unwrap();

    let pins = peripherals.pins;
    let spi2 = peripherals.spi2;

    let ledc = peripherals.ledc;

    let mut display = prepare_display(
        spi2,
        pins.gpio7,
        Some(pins.gpio2), //not true but we need to make the compiler happy
        pins.gpio6,
        Some(pins.gpio10),
        pins.gpio3,
        pins.gpio4,
        pins.gpio11, // not true
        ledc.timer0,
        ledc.channel0,
    );

    let button_a = PinDriver::input(pins.gpio8).unwrap();
    let button_b = PinDriver::input(pins.gpio9).unwrap();

    let mut delay = Ets;

    display.hard_reset(&mut delay).unwrap();
    display.init(&mut delay).unwrap();
    display
        .set_orientation(display_driver::Orientation::LandscapeSwapped)
        .unwrap();

    display.set_address_window(0, 0, 319, 239).unwrap();

    let mut raw_framebuffer_0 = Box::new([0u16; 240 * 320]);

    let mut dma_ready_framebuffer =
        DmaReadyFramebuffer::<320, 240>::new(raw_framebuffer_0.as_mut_ptr() as *mut c_void, true);

    let mut teapot = K3dMesh::new(embed_stl!("src/Teapot_low.stl"));
    teapot.set_position(0.0, 0.0, 0.0);
    teapot.set_render_mode(RenderMode::Lines);
    teapot.set_scale(0.1);
    teapot.set_color(Rgb565::CSS_BLUE);

    let mut engine = K3dengine::new(320, 240);
    engine.camera.set_position(Point3::new(0.0, 0.0, -3.0));
    engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));
    engine.camera.set_fovy(PI / 5.0);

    let mut rot = 0.0;
    let mut pos = 0.0;

    let mut perf = PerformanceCounter::new();
    perf.only_fps(false);

    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_WHITE);

    loop {
        perf.start_of_frame();
        raw_framebuffer_0.fill(0);
        perf.add_measurement("clear");

        engine.render([&teapot], |p| {
            //println!("draw {:?}", &p);
            draw(p, &mut dma_ready_framebuffer);
        });

        perf.add_measurement("render");

        Text::new(perf.get_text(), Point::new(20, 20), text_style)
            .draw(&mut dma_ready_framebuffer)
            .unwrap();

        display
            .eat_framebuffer(dma_ready_framebuffer.as_slice())
            .unwrap();

        perf.add_measurement("draw");

        teapot.set_attitude(rot, rot * 2.0, 0.0);
        rot += 0.03;

        if button_a.is_high() && button_b.is_low() {
            pos += 0.05;
        }

        if button_a.is_low() && button_b.is_high() {
            pos -= 0.05;
        }

        engine
            .camera
            .set_position(Point3::new(0.0, 0.0, -3.0 + pos));

        perf.add_measurement("update");

        perf.print();
    }
}

#[allow(clippy::too_many_arguments)]
pub fn prepare_display<SPI: SpiAnyPins>(
    spi: impl Peripheral<P = SPI> + 'static,
    sdo: impl Peripheral<P = impl OutputPin> + 'static,
    sdi: Option<impl Peripheral<P = impl InputPin> + 'static>,
    sclk: impl Peripheral<P = impl OutputPin> + 'static,
    cs: Option<impl Peripheral<P = impl OutputPin> + 'static>,
    rst: impl Peripheral<P = impl OutputPin> + 'static,
    dc: impl Peripheral<P = impl OutputPin> + 'static,
    bl: impl Peripheral<P = impl OutputPin> + 'static,
    ledc_timer: impl Peripheral<P = impl LedcTimer> + 'static,
    ledc_channel: impl Peripheral<P = impl LedcChannel> + 'static,
) -> display_driver::ST7789<
    SPIInterface<
        SpiDeviceDriver<'static, SpiDriver<'static>>,
        PinDriver<'static, impl OutputPin, Output>,
    >,
    esp_idf_hal::gpio::PinDriver<'static, impl OutputPin, esp_idf_hal::gpio::Output>,
    esp_idf_hal::gpio::PinDriver<'static, impl OutputPin, esp_idf_hal::gpio::Output>,
> {
    let config = esp_idf_hal::spi::config::Config::new()
        .baudrate(80.MHz().into())
        .data_mode(esp_idf_hal::spi::config::MODE_0)
        .queue_size(1);
    let device = SpiDeviceDriver::new_single(
        spi,
        sclk,
        sdo,
        sdi,
        cs,
        &SpiDriverConfig::new().dma(Dma::Auto(4096)),
        &config,
    )
    .unwrap();

    let pin_dc = PinDriver::output(dc).unwrap();

    let spi_interface = SPIInterface::new(device, pin_dc);

    // let ledc_config = esp_idf_svc::hal::ledc::config::TimerConfig::new().frequency(25.kHz().into());
    // let timer = LedcTimerDriver::new(ledc_timer, &ledc_config).unwrap();

    // let backlight_pwm = LedcDriver::new(ledc_channel, timer, bl).unwrap();
    // backlight_pwm.set_duty(backlight_pwm.get_max_duty()).unwrap();

    let rst_pin = PinDriver::output(rst).unwrap();
    let bl_pin = PinDriver::output(bl).unwrap();

    display_driver::ST7789::new(spi_interface, Some(rst_pin), Some(bl_pin))
}
