use instruction::{Instruction, Opcode};
use std::fs::File;
use std::io::Read;
use std::path::Path;
use termios::*;

mod instruction;

enum Registers {
    R0 = 0,
    _R1,
    _R2,
    _R3,
    _R4,
    _R5,
    _R6,
    R7,
    PC,
    COUNT,
}

pub struct Flags {
    n: bool,
    z: bool,
    p: bool,
}

impl Flags {
    pub fn update(&mut self, reg: u16) {
        match reg {
            0x0000 => {
                self.n = false;
                self.z = true;
                self.p = false
            }
            0x0001..=0x7FFF => {
                self.n = false;
                self.p = true;
                self.z = false;
            }
            0x8000..=0xFFFF => {
                self.n = true;
                self.p = false;
                self.z = false
            }
        };
    }
}

impl From<u8> for Flags {
    fn from(flags: u8) -> Flags {
        Flags {
            n: ((flags & 0x4) >> 2) == 1,
            z: ((flags & 0x2) >> 1) == 1,
            p: (flags & 0x1) == 1,
        }
    }
}

enum TrapCode {
    Getc(),
    Out(),
    Puts(),
    In(),
    Putsp(),
    Halt(),
    Unknown(),
}

impl From<u8> for TrapCode {
    fn from(vector: u8) -> TrapCode {
        match vector {
            0x20 => TrapCode::Getc(),
            0x21 => TrapCode::Out(),
            0x22 => TrapCode::Puts(),
            0x23 => TrapCode::In(),
            0x24 => TrapCode::Putsp(),
            0x25 => TrapCode::Halt(),
            _ => TrapCode::Unknown(),
        }
    }
}

enum MemoryMappedRegisters {
    KBSR = 0xFE00,
    KBDR = 0xFE02,
}

fn mem_read(mem: &mut [u16], addr: u16) -> u16 {
    if addr == MemoryMappedRegisters::KBSR as u16 {
        let mut buffer = [0; 1];
        std::io::stdin().read_exact(&mut buffer).unwrap();
        if buffer[0] != 0 {
            mem[MemoryMappedRegisters::KBSR as usize] = 1 << 15;
            mem[MemoryMappedRegisters::KBDR as usize] = buffer[0] as u16
        } else {
            mem[MemoryMappedRegisters::KBSR as usize] = 0
        }
    }
    mem[addr as usize]
}

fn mem_write(mem: &mut [u16], addr: u16, val: u16) {
    mem[addr as usize] = val
}

fn abort() {
    std::process::exit(1);
}

fn get16(mem: &[u8], ind: usize) -> u16 {
    ((mem[ind] as u16) << 8) + mem[ind + 1] as u16
}

// https://doc.rust-lang.org/rust-by-example/std_misc/file/open.html
fn read_image(mem: &mut [u16], image_path: &str) -> u32 {
    let path = Path::new(image_path);
    // println!("[*] Loading {}", path.to_str().unwrap());
    let mut file = File::open(&path).expect("Couldn't open file.");

    const SIZE: u32 = std::u16::MAX as u32 * 2 - 2;
    let mut mem_buffer: [u8; SIZE as usize] = [0; SIZE as usize];
    file.read(&mut mem_buffer).expect("Couldn't read file.");
    let length = file.metadata().unwrap().len();
    // println!("[*] File length {}", length);

    let base = get16(&mem_buffer, 0) as usize;
    for i in (2..length).step_by(2) {
        // println!("{}",i);
        mem[base + (i / 2 - 1) as usize] = get16(&mem_buffer, i as usize);
    }
    // println!("{:?}", &mem[0x3000..0x4000]);
    length as u32
}

fn run() {
    println!("run");
    let mut memory: [u16; std::u16::MAX as usize] = [0; std::u16::MAX as usize];

    read_image(&mut memory, "sample_apps/2048.obj");

    let mut regs: [u16; Registers::COUNT as usize] = [0; Registers::COUNT as usize];
    let mut flags = Flags::from(0);

    let pc_start: u16 = 0x3000;
    regs[Registers::PC as usize] = pc_start;

    let mut running: bool = true;

    while running {
        let opcode = Opcode::from(mem_read(&mut memory, regs[Registers::PC as usize]));
        let instruction = Instruction::new(opcode).unwrap();
        regs[Registers::PC as usize] += 1;

        match instruction {
            Instruction::Br(n, z, p, offset) => {
                if (n & flags.n) || (z & flags.z) || (p & flags.p) {
                    regs[Registers::PC as usize] += offset
                }
            }
            Instruction::AddReg(dr, sr1, sr2) => {
                regs[dr] = regs[sr1] + regs[sr2];
                flags.update(regs[dr])
            }
            Instruction::AddImm(dr, sr1, imm5) => {
                regs[dr] = regs[sr1] + (imm5 as u16);
                flags.update(regs[dr])
            }
            Instruction::AndReg(dr, sr1, sr2) => {
                regs[dr] = regs[sr1] & regs[sr2];
                flags.update(regs[dr])
            }
            Instruction::AndImm(dr, sr1, imm5) => {
                regs[dr] = regs[sr1] & (imm5 as u16);
                flags.update(regs[dr])
            }
            Instruction::Jmp(base_r) => regs[Registers::PC as usize] = regs[base_r],
            Instruction::Jsr(offset) => {
                regs[Registers::R7 as usize] = regs[Registers::PC as usize];
                regs[Registers::PC as usize] += offset
            }
            Instruction::Jsrr(base_r) => {
                regs[Registers::R7 as usize] = regs[Registers::PC as usize];
                regs[Registers::PC as usize] = regs[base_r]
            }
            Instruction::Ld(dr, offset) => {
                regs[dr] = mem_read(&mut memory, regs[Registers::PC as usize] + offset);
                flags.update(regs[dr])
            }
            Instruction::Ldi(dr, offset) => {
                let addr = mem_read(&mut memory, regs[Registers::PC as usize] + offset);
                regs[dr] = mem_read(&mut memory, addr);
                flags.update(regs[dr])
            }
            Instruction::Ldr(dr, base_r, offset) => {
                regs[dr] = mem_read(&mut memory, regs[base_r] + offset);
                flags.update(regs[dr])
            }
            Instruction::Lea(dr, offset) => {
                regs[dr] = regs[Registers::PC as usize] + offset;
                flags.update(regs[dr])
            }
            Instruction::Not(dr, sr) => {
                regs[dr] = !regs[sr];
                flags.update(regs[dr])
            }
            Instruction::Rti() => abort(),
            Instruction::St(sr, offset) => {
                mem_write(&mut memory, regs[Registers::PC as usize] + offset, regs[sr]);
            }
            Instruction::Sti(_sr, _offset) => {
                abort()
                // mem_write(&mut memory, regs[Registers::PC as usize] + offset, regs[sr])
            }
            Instruction::Str(sr, base_r, offset) => {
                mem_write(&mut memory, regs[base_r] + offset, regs[sr])
            }
            Instruction::Trap(trap_vector) => match TrapCode::from(trap_vector) {
                TrapCode::Getc() => {
                    let mut buffer = [0; 1];
                    std::io::stdin().read_exact(&mut buffer).unwrap();
                    regs[Registers::R0 as usize] = buffer[0] as u16
                }
                TrapCode::Out() => {
                    let c = regs[Registers::R0 as usize] as u8;
                    print!("{}", c as char);
                }
                TrapCode::Puts() => {
                    for c in &memory[regs[Registers::R0 as usize] as usize..] {
                        let c8 = (c & 0xFF) as u8;
                        if c8 != 0x00 {
                            print!("{}", c8 as char);
                        } else {
                            break;
                        }
                    }
                }
                TrapCode::In() => {
                    print!("Enter a character: ");
                    regs[Registers::R0 as usize] = std::io::stdin()
                        .bytes()
                        .next()
                        .and_then(|result| result.ok())
                        .map(|byte| byte as u16)
                        .unwrap();
                    // println!("[*] IN");
                }
                // OUTPUT A BYTE STRING
                TrapCode::Putsp() => {
                    for c in &memory[regs[Registers::R0 as usize] as usize..] {
                        let b1 = (*c >> 8) as u8;
                        let b2 = (*c & 0xff) as u8;
                        if b1 != 0 {
                            print!("{}", b1 as char);
                            if b2 != 0 {
                                print!("{}", b2 as char);
                            }
                        }
                    }
                    // println!("[*] PUTSP");
                }
                TrapCode::Halt() => {
                    println!("[!] HALT");
                    running = false;
                }
                TrapCode::Unknown() => {
                    println!("[!] UNKNOWN TRAPCODE");
                    abort();
                }
            },
            Instruction::Reserved() => {
                // println!("[*] RES");
            }
        }
    }
}

fn main() {
    // https://stackoverflow.com/questions/26321592/how-can-i-read-one-character-from-stdin-without-having-to-hit-enter
    let stdin = 0; // couldn't get std::os::unix::io::FromRawFd to work
                   // on /dev/stdin or /dev/tty
    let termios = Termios::from_fd(stdin).unwrap();
    let mut new_termios = termios.clone(); // make a mutable copy of termios
                                           // that we will modify
    new_termios.c_iflag &= IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON;
    new_termios.c_lflag &= !(ICANON | ECHO); // no echo and canonical mode
    tcsetattr(stdin, TCSANOW, &mut new_termios).unwrap();

    run();

    tcsetattr(stdin, TCSANOW, &termios).unwrap(); // reset the stdin to
                                                  // original termios data
}
