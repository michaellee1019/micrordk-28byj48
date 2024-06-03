use micro_rdk::common::{
    config::ConfigType,
    board::Board,
    motor::{Motor,MotorType},
    registry::{get_board_from_dependencies, Dependency},
};

use std::sync::{
    Arc, Mutex
};

use std::collections::HashMap;

use micro_rdk::common::status::Status;
use micro_rdk::common::config::Kind;
use micro_rdk::common::actuator::Actuator;
use micro_rdk::common::motor::MotorSupportedProperties;
use std::thread::sleep;
use std::time;

const HALF_STEP_FWD: [[bool;4];8] = [
    [false, false, false, true],
    [false, false, true, true],
    [false, false, true, false],
    [false, true, true, false],
    [false, true, false, false],
    [true, true, false, false],
    [true, false, false, false],
    [true, false, false, true],
];

const HALF_STEP_REV: [[bool;4];8] = [
    [true, false, false, true],
    [true, false, false, false],
    [true, true, false, false],
    [false, true, false, false],
    [false, true, true, false],
    [false, false, true, false],
    [false, false, true, true],
    [false, false, false, true],
];

// Represents a motor using a A, B, and PWM pins
#[derive(DoCommand)]
pub struct Stepper<B> {
    board: B,
    in_1: i32,
    in_2: i32,
    in_3: i32,
    in_4: i32,
}

impl<B> Stepper<B>
where
    B: Board,
{
    pub(crate) fn new(
        in_1: i32,
        in_2: i32,
        in_3: i32,
        in_4: i32,
        board: B,
    ) -> anyhow::Result<Self> {
        let mut res = Self {
            board,
            in_1,
            in_2,
            in_3,
            in_4,
        };
        Ok(res)
    }

    pub fn from_config(cfg: ConfigType, deps: Vec<Dependency>) -> anyhow::Result<MotorType> {
        let board = get_board_from_dependencies(deps)
        .ok_or_else(|| anyhow::Error::msg("stepper motor requires a board in its dependencies"))?;

        let in_1 = cfg.get_attribute::<i32>("in_1")?;
        let in_2 = cfg.get_attribute::<i32>("in_2")?;
        let in_3 = cfg.get_attribute::<i32>("in_3")?;
        let in_4 = cfg.get_attribute::<i32>("in_4")?;

        return Ok(Arc::new(Mutex::new(Stepper::new(
            in_1, in_2, in_3, in_4, board,
        )?)));

    }

    fn step(&mut self, count:u32, forwards:bool) -> anyhow::Result<()>{
        for _ in 0..count {
            let iter = if forwards { HALF_STEP_FWD.iter() } else { HALF_STEP_REV.iter() };
            for step in iter {
                self.board.set_gpio_pin_level(self.in_1,step[0])?;
                self.board.set_gpio_pin_level(self.in_2,step[1])?;
                self.board.set_gpio_pin_level(self.in_3,step[2])?;
                self.board.set_gpio_pin_level(self.in_4,step[3])?;
                sleep(time::Duration::from_millis(2));
            }
        }
        self.board.set_gpio_pin_level(self.in_1,false)?;
        self.board.set_gpio_pin_level(self.in_2,false)?;
        self.board.set_gpio_pin_level(self.in_3,false)?;
        self.board.set_gpio_pin_level(self.in_4,false)?;
        Ok(())
    }
}

impl<B> Motor for Stepper<B>
where
    B: Board,
{
    fn set_power(&mut self, pct: f64) -> anyhow::Result<()> {
        let mut steps = pct.abs() * 512.0;
        if pct < 0.0 {
            self.step(steps as u32, false)?;
        }
        else {
            self.step(steps as u32, true)?;
        }

        Ok(())
    }

    fn get_position(&mut self) -> anyhow::Result<i32> {
        anyhow::bail!("position reporting not supported without an encoder")
    }

    fn go_for(
        &mut self,
        rpm: f64,
        revolutions: f64,
    ) -> anyhow::Result<Option<std::time::Duration>> {
        anyhow::bail!("no go_for :(")
    }

    fn get_properties(&mut self) -> MotorSupportedProperties {
        MotorSupportedProperties {
            position_reporting: false,
        }
    }
}

impl<B> Status for Stepper<B>
where
    B: Board,
{
    fn get_status(&self) -> anyhow::Result<Option<micro_rdk::google::protobuf::Struct>> {
        let mut hm = HashMap::new();
        Ok(Some(micro_rdk::google::protobuf::Struct { fields: hm }))
    }
}

impl<B> Actuator for Stepper<B>
where
    B: Board,
{
    fn stop(&mut self) -> anyhow::Result<()> {
        self.set_power(0.0)
    }
    fn is_moving(&mut self) -> anyhow::Result<bool> {
        Ok(false)
    }
}
