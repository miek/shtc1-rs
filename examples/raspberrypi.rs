extern crate linux_embedded_hal as hal;
extern crate shtc1;

use hal::{Delay, I2cdev};
use shtc1::SHTC1;

fn main() {
    println!("Hello, SHTC1!");

    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut shtc1 = SHTC1::new(dev, Delay);

    println!("ID raw: {:?}", shtc1.read_id().unwrap());
    loop {
        let m = shtc1.measure().unwrap();
        let temp = m.temperature as f32 / 100.0;
        let humidity = m.humidity as f32 / 100.0;
        println!("Temp: {:.2} Humidity: {:.2}%", temp, humidity);
    }
}
