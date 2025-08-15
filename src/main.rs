use embedded_gfx::{draw::draw, framebuffer::DmaReadyFramebuffer, mesh::{K3dMesh, RenderMode}, K3dengine};
use embedded_graphics::{pixelcolor::Rgb565, prelude::{IntoStorage, Point, WebColors}};
use esp_idf_svc::hal::peripheral::Peripheral;
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

mod display_driver;
mod swapchain;

use display_interface_spi::SPIInterface;
use embedded_gfx::mesh::Geometry;
use load_stl::embed_stl;
use nalgebra::{Point2, Point3};

use crate::{display_driver::FramebufferTarget, swapchain::DoubleBuffer};

#[allow(clippy::approx_constant)]
fn main() {
    esp_idf_svc::sys::link_patches();

    // esp_idf_hal::i2s::I2sDriver::new_std_tx(i2s, config, bclk, dout, mclk, ws)
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
        pins.gpio9, // not true
        ledc.timer0,
        ledc.channel0,
    );

    let mut delay = Ets;

    let hor_res = 320;
    let ver_res = 240;
    let boh1 = 40;
    let boh2 = 53;

    display.hard_reset(&mut delay).unwrap();
    display.init(&mut delay).unwrap();
    display
        .set_orientation(display_driver::Orientation::Landscape)
        .unwrap();
    //display
    //    .set_address_window(0 + boh1, 0 + boh2, hor_res - 1 + boh1, ver_res + boh2)
    //    .unwrap();

         display
       .set_address_window(0, 0, 319, 239)
         .unwrap();

    let mut raw_framebuffer_0 = Box::new([0u16; 240 * 320]);

    let mut dma_ready_framebuffer = DmaReadyFramebuffer::<320, 240>::new(raw_framebuffer_0.as_mut_ptr() as *mut c_void, false);

    let mut teapot = K3dMesh::new(embed_stl!("src/Teapot_low.stl"));
    teapot.set_position(0.0, 0.0, 0.0);
    teapot.set_render_mode(RenderMode::Lines);
    teapot.set_scale(0.1);
    teapot.set_color(Rgb565::CSS_BLUE);

    let mut engine = K3dengine::new(320, 240);
    engine.camera.set_position(Point3::new(0.0, -0.0, -3.0));
    engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));
    engine.camera.set_fovy(PI / 5.0);

    let mut rot = 0.0;

    loop {
        raw_framebuffer_0.fill(Rgb565::CSS_RED.into_storage());


        engine.render([&teapot], |p| {
            //println!("draw {:?}", &p);
            draw(p, &mut dma_ready_framebuffer);
        });
        display.eat_framebuffer(dma_ready_framebuffer.as_slice()).unwrap();

        teapot.set_attitude(rot, rot*2.0, 0.0);
        rot += 0.03;

                
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
        .baudrate(20.MHz().into())
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
