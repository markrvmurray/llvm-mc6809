#[derive(Debug)]
pub struct Memory {
    ram: [u8; 56 * 1024],
    rom: [u8; 8 * 1024 - 512],
}

impl Memory {
    // A public constructor method
    pub fn new() -> Memory {
        Memory {
            ram: [0; 56 * 1024],
            rom: [0; 8 * 1024 - 512],
        }
    }

    pub fn write(&mut self, address: &u16, value: u8) {
        if *address < 56 * 1024u16 {
            self.ram[*address as usize] = value;
        } else if *address >= 56 * 1024u16 && *address < 56 * 1024u16 + 512 {
            // Handle I/O here
        }
    }

    pub fn rom_write(&mut self, address: &u16, buffer: &[u8]) {
        assert!(*address >= 56 * 1024u16);
        assert!((*address as usize) + buffer.len() < 0x10000usize);
    }

    pub fn read8(&self, address: &u16) -> u8 {
        if *address < 56 * 1024u16 {
            return self.ram[*address as usize];
        } else if *address < 56 * 1024u16 + 512 {
            // Handle I/O here
            return 0x00u8;
        } else {
            return self.rom[(*address - 56 * 1024u16 - 512) as usize];
        }
    }

    pub fn read16(&self, address: &u16) -> u16 {
        let mut temp: u16 = self.read8(address) as u16;
        temp = (temp << 8) | self.read8(&(address + 1)) as u16;
        temp
    }

    pub fn read32(&self, address: &u16) -> u32 {
        let mut temp: u32 = self.read8(address) as u32;
        temp = (temp << 8) + self.read8(&(address + 1)) as u32;
        temp = (temp << 8) + self.read8(&(address + 2)) as u32;
        temp = (temp << 8) + self.read8(&(address + 3)) as u32;
        temp
    }
}

#[derive(Debug)]
struct Register {
    a: u8,
    b: u8,
    e: u8,
    f: u8,
    x: u16,
    y: u16,
    u: u16,
    s: u16,
    pc: u16,
    v: u16,
    zero: u16,
    dp: u8,
    cc: u8,
    mode: u8,
}

impl Register {
    pub fn new() -> Register {
        Register {
            a: 0x00u8,
            b: 0x00u8,
            e: 0x00u8,
            f: 0x00u8,
            cc: 0x00u8,
            mode: 0x00u8,
            dp: 0x00u8,
            x: 0x0000u16,
            y: 0x0000u16,
            u: 0x0000u16,
            s: 0x0000u16,
            zero: 0x0000u16,
            pc: 0x0000u16,
            v: 0xBCC5,
        }
    }

    pub fn reset(&mut self, pc: u16) {
        self.a = 0x00u8;
        self.b = 0x00u8;
        self.e = 0x00u8;
        self.f = 0x00u8;
        self.cc = 0x00u8;
        self.mode = 0x00u8;
        self.dp = 0x00u8;
        self.x = 0x0000u16;
        self.y = 0x0000u16;
        self.u = 0x0000u16;
        self.s = 0x0000u16;
        self.zero = 0x0000u16;
        self.pc = pc;
    }

    pub fn pc_increment(&mut self) {
        self.pc += 1;
    }
}

#[derive(Debug)]
enum IndexReg {
    X,
    Y,
    U,
    S,
}

#[derive(Debug)]
enum Reg {
    D,
    X,
    Y,
    U,
    S,
    PC,
    W,
    V,
    A,
    B,
    CC,
    DP,
    Zero0,
    Zero1,
    E,
    F,
}

#[derive(Debug)]
enum Cond {
    HI = 2,
    LS,
    HS,
    LO,
    NE,
    EQ,
    VC,
    VS,
    PL,
    MI,
    GE,
    LT,
    GT,
    LE,
}

#[derive(Debug)]
enum IndexMode {
    ZeroOffs { idx: IndexReg },
    Imm5Offs { idx: IndexReg, offset: i8 },
    Imm8Offs { idx: IndexReg, offset: i8 },
    Imm16Offs { idx: IndexReg, offset: i16 },

    ZeroOffsW,
    Imm16OffsW { offset: i16 },

    AccA8Offs { idx: IndexReg },
    AccB8Offs { idx: IndexReg },
    AccD16Offs { idx: IndexReg },
    AccE8Offs { idx: IndexReg },
    AccF8Offs { idx: IndexReg },
    AccW16Offs { idx: IndexReg },

    PostInc1 { idx: IndexReg },
    PostInc2 { idx: IndexReg },
    PreDec1 { idx: IndexReg },
    PreDec2 { idx: IndexReg },

    PostInc2W,
    PreDec2W,

    Imm8OffsPC { offset: i8 },
    Imm16OffsPC { offset: i16 },

    IndirZeroOffs { idx: IndexReg },
    IndirImm8Offs { idx: IndexReg, offset: i8 },
    IndirImm16Offs { idx: IndexReg, offset: i16 },

    IndirZeroOffsW,
    IndirImm16OffsW { offset: i16 },

    IndirAccA8Offs { idx: IndexReg },
    IndirAccB8Offs { idx: IndexReg },
    IndirAccD16Offs { idx: IndexReg },
    IndirAccE8Offs { idx: IndexReg },
    IndirAccF8Offs { idx: IndexReg },
    IndirAccW16Offs { idx: IndexReg },

    IndirPostInc2 { idx: IndexReg },
    IndirPreDec2 { idx: IndexReg },

    IndirPostInc2W,
    IndirPreDec2W,

    IndirImm8OffsPC { offset: i8 },
    IndirImm16OffsPC { offset: i16 },

    IndirExtended { addr: u16 },
}

impl IndexMode {
    // A public constructor method
    pub fn new(indexbyte: u8, byte1: u8, byte2: u8) -> IndexMode {
        let iregbits = (indexbyte >> 5) & 0b0000_0011;
        let ireg = match iregbits {
            0b0000_0000 => IndexReg::X,
            0b0000_0001 => IndexReg::Y,
            0b0000_0010 => IndexReg::U,
            0b0000_0011 => IndexReg::S,
            _ => unreachable!(),
        };
        match indexbyte {
            0b1000_0100 | 0b1010_0100 | 0b1100_0100 | 0b1110_0100 => {
                IndexMode::ZeroOffs { idx: ireg }
            }
            0b0000_0000..=0b0111_1111 => {
                let offset: i8 = if (indexbyte & 0b0001_1111) > 15 {
                    (indexbyte & 0b0000_1111) | 0b1111_0000
                } else {
                    indexbyte & 0b0000_1111
                } as i8;
                IndexMode::Imm5Offs {
                    idx: ireg,
                    offset: offset,
                }
            }
            0b1000_1000 | 0b1010_1000 | 0b1100_1000 | 0b1110_1000 => {
                let offset = byte1 as i8;
                IndexMode::Imm8Offs {
                    idx: ireg,
                    offset: offset,
                }
            }
            0b1000_1001 | 0b1010_1001 | 0b1100_1001 | 0b1110_1001 => {
                let offset: i16 = (((byte1 as u16) << 8) + byte2 as u16) as i16;
                IndexMode::Imm16Offs {
                    idx: ireg,
                    offset: offset,
                }
            }
            0b1000_1111 => IndexMode::ZeroOffsW,
            0b1010_1111 => {
                let offset: i16 = (((byte1 as u16) << 8) + byte2 as u16) as i16;
                IndexMode::Imm16OffsW { offset: offset }
            }
            0b1000_0110 | 0b1010_0110 | 0b1100_0110 | 0b1110_0110 => {
                IndexMode::AccA8Offs { idx: ireg }
            }
            0b1000_0101 | 0b1010_0101 | 0b1100_0101 | 0b1110_0101 => {
                IndexMode::AccB8Offs { idx: ireg }
            }
            0b1000_1011 | 0b1010_1011 | 0b1100_1011 | 0b1110_1011 => {
                IndexMode::AccD16Offs { idx: ireg }
            }
            0b1000_0111 | 0b1010_0111 | 0b1100_0111 | 0b1110_0111 => {
                IndexMode::AccE8Offs { idx: ireg }
            }
            0b1000_1010 | 0b1010_1010 | 0b1100_1010 | 0b1110_1010 => {
                IndexMode::AccF8Offs { idx: ireg }
            }
            0b1000_1110 | 0b1010_1110 | 0b1100_1110 | 0b1110_1110 => {
                IndexMode::AccW16Offs { idx: ireg }
            }
            0b1000_0000 | 0b1010_0000 | 0b1100_0000 | 0b1110_0000 => {
                IndexMode::PostInc1 { idx: ireg }
            }
            0b1000_0001 | 0b1010_0001 | 0b1100_0001 | 0b1110_0001 => {
                IndexMode::PostInc2 { idx: ireg }
            }
            0b1000_0010 | 0b1010_0010 | 0b1100_0010 | 0b1110_0010 => {
                IndexMode::PreDec1 { idx: ireg }
            }
            0b1000_0011 | 0b1010_0011 | 0b1100_0011 | 0b1110_0011 => {
                IndexMode::PreDec2 { idx: ireg }
            }
            0b1100_1111 => IndexMode::PostInc2W,
            0b1110_1111 => IndexMode::PreDec2W,
            0b1000_1100 | 0b1010_1100 | 0b1100_1100 | 0b1110_1100 => {
                let offset = byte1 as i8;
                IndexMode::Imm8OffsPC { offset: offset }
            }
            0b1000_1101 | 0b1010_1101 | 0b1100_1101 | 0b1110_1101 => {
                let offset: i16 = (((byte1 as u16) << 8) + byte2 as u16) as i16;
                IndexMode::Imm16OffsPC { offset: offset }
            }

            0b1001_0100 | 0b1011_0100 | 0b1101_0100 | 0b1111_0100 => {
                IndexMode::IndirZeroOffs { idx: ireg }
            }
            0b1001_1000 | 0b1011_1000 | 0b1101_1000 | 0b1111_1000 => {
                let offset = byte1 as i8;
                IndexMode::IndirImm8Offs {
                    idx: ireg,
                    offset: offset,
                }
            }
            0b1001_1001 | 0b1011_1001 | 0b1101_1001 | 0b1111_1001 => {
                let offset: i16 = (((byte1 as u16) << 8) + byte2 as u16) as i16;
                IndexMode::IndirImm16Offs {
                    idx: ireg,
                    offset: offset,
                }
            }
            0b1001_0000 => IndexMode::IndirZeroOffsW,
            0b1011_0000 => {
                let offset: i16 = (((byte1 as u16) << 8) + byte2 as u16) as i16;
                IndexMode::IndirImm16OffsW { offset: offset }
            }
            0b1001_0110 | 0b1011_0110 | 0b1101_0110 | 0b1111_0110 => {
                IndexMode::IndirAccA8Offs { idx: ireg }
            }
            0b1001_0101 | 0b1011_0101 | 0b1101_0101 | 0b1111_0101 => {
                IndexMode::IndirAccB8Offs { idx: ireg }
            }
            0b1001_1011 | 0b1011_1011 | 0b1101_1011 | 0b1111_1011 => {
                IndexMode::IndirAccD16Offs { idx: ireg }
            }
            0b1001_0111 | 0b1011_0111 | 0b1101_0111 | 0b1111_0111 => {
                IndexMode::IndirAccE8Offs { idx: ireg }
            }
            0b1001_1010 | 0b1011_1010 | 0b1101_1010 | 0b1111_1010 => {
                IndexMode::IndirAccF8Offs { idx: ireg }
            }
            0b1001_1110 | 0b1011_1110 | 0b1101_1110 | 0b1111_1110 => {
                IndexMode::IndirAccW16Offs { idx: ireg }
            }
            0b1001_0001 | 0b1011_0001 | 0b1101_0001 | 0b1111_0001 => {
                IndexMode::IndirPostInc2 { idx: ireg }
            }
            0b1001_0010 | 0b1011_0010 | 0b1101_0010 | 0b1111_0010 => {
                unreachable!()
            }
            0b1001_0011 | 0b1011_0011 | 0b1101_0011 | 0b1111_0011 => {
                IndexMode::IndirPreDec2 { idx: ireg }
            }
            0b1101_0000 => IndexMode::IndirPostInc2W,
            0b1111_0000 => IndexMode::IndirPreDec2W,
            0b1001_1100 | 0b1011_1100 | 0b1101_1100 | 0b1111_1100 => {
                let offset = byte1 as i8;
                IndexMode::IndirImm8OffsPC { offset: offset }
            }
            0b1001_1101 | 0b1011_1101 | 0b1101_1101 | 0b1111_1101 => {
                let offset: i16 = (((byte1 as u16) << 8) + byte2 as u16) as i16;
                IndexMode::IndirImm16OffsPC { offset: offset }
            }
            0b1001_1111 => {
                let address: u16 = ((byte1 as u16) << 8) + byte2 as u16;
                IndexMode::IndirExtended { addr: address }
            }
            0b1011_1111 | 0b1101_1111 | 0b1111_1111 => {
                unreachable!()
            }
        }
    }
}

//#[derive(Debug)]
//struct Addr8 {
//    addr: u8,
//}

//#[derive(Debug)]
//struct Addr16 {
//    addr: u16,
//}

//#[derive(Debug)]
//struct Imm8 {
//    val: u8,
//}

//#[derive(Debug)]
//struct Imm16 {
//    val: u16,
//}

#[derive(Debug)]
struct BitSet {
    val: u8,
}

impl BitSet {
    pub fn new(bitsetbyte: u8) -> BitSet {
        BitSet { val: bitsetbyte }
    }
}

#[derive(Debug)]
struct RegPair {
    pair: u8,
}

impl RegPair {
    pub fn new(regpairbyte: u8) -> RegPair {
        RegPair { pair: regpairbyte }
    }
}

#[derive(Debug)]
enum Instruction {
    // MRVM START MARKER 1
    NEGd(u8),
    OIMid(u8, u8),
    AIMid(u8, u8),
    COMd(u8),
    LSRd(u8),
    EIMid(u8, u8),
    RORd(u8),
    ASRd(u8),
    ASLd(u8),
    ROLd(u8),
    DECd(u8),
    TIMid(u8, u8),
    INCd(u8),
    TSTd(u8),
    JMPd(u8),
    CLRd(u8),
    NOPx,
    SYNCx,
    SEXWx,
    HCFx,
    LBRAlb(i16),
    LBSRlb(i16),
    DAAx,
    ORCCi8(u8),
    ANDCCi8(u8),
    SEXx,
    EXGp(RegPair),
    TFRp(RegPair),
    BRAb(i8),
    BRNb(i8),
    Bbc(Cond, i8),
    LEAXi(IndexMode),
    LEAYi(IndexMode),
    LEASi(IndexMode),
    LEAUi(IndexMode),
    PSHSs(u8),
    PULSs(u8),
    PSHUs(u8),
    PULUs(u8),
    RTSr,
    ABXx,
    RTIr,
    CWAIi8(u8),
    MULx,
    RESETx,
    SWIx,
    NEGAa,
    COMAa,
    LSRAa,
    RORAa,
    ASRAa,
    ASLAa,
    ROLAa,
    DECAa,
    INCAa,
    TSTAa,
    CLRAa,
    NEGBa,
    COMBa,
    LSRBa,
    RORBa,
    ASRBa,
    ASLBa,
    ROLBa,
    DECBa,
    INCBa,
    TSTBa,
    CLRBa,
    NEGi(IndexMode),
    OIMii(u8, IndexMode),
    AIMii(u8, IndexMode),
    COMi(IndexMode),
    LSRi(IndexMode),
    EIMii(u8, IndexMode),
    RORi(IndexMode),
    ASRi(IndexMode),
    ASLi(IndexMode),
    ROLi(IndexMode),
    DECi(IndexMode),
    TIMii(u8, IndexMode),
    INCi(IndexMode),
    TSTi(IndexMode),
    JMPi(IndexMode),
    CLRi(IndexMode),
    NEGe(u16),
    OIMie(u8, u16),
    AIMie(u8, u16),
    COMe(u16),
    LSRe(u16),
    EIMie(u8, u16),
    RORe(u16),
    ASRe(u16),
    ASLe(u16),
    ROLe(u16),
    DECe(u16),
    TIMie(u8, u16),
    INCe(u16),
    TSTe(u16),
    JMPe(u16),
    CLRe(u16),
    SUBAi8(u8),
    CMPAi8(u8),
    SBCAi8(u8),
    SUBDi16(u16),
    ANDAi8(u8),
    BITAi8(u8),
    LDAi8(u8),
    EORAi8(u8),
    ADCAi8(u8),
    ORAi8(u8),
    ADDAi8(u8),
    CMPXi16(u16),
    BSRb(i8),
    LDXi16(u16),
    SUBAd(u8),
    CMPAd(u8),
    SBCAd(u8),
    SUBDd(u8),
    ANDAd(u8),
    BITAd(u8),
    LDAd(u8),
    STAd(u8),
    EORAd(u8),
    ADCAd(u8),
    ORAd(u8),
    ADDAd(u8),
    CMPXd(u8),
    JSRd(u8),
    LDXd(u8),
    STXd(u8),
    SUBAi(IndexMode),
    CMPAi(IndexMode),
    SBCAi(IndexMode),
    SUBDi(IndexMode),
    ANDAi(IndexMode),
    BITAi(IndexMode),
    LDAi(IndexMode),
    STAi(IndexMode),
    EORAi(IndexMode),
    ADCAi(IndexMode),
    ORAi(IndexMode),
    ADDAi(IndexMode),
    CMPXi(IndexMode),
    JSRi(IndexMode),
    LDXi(IndexMode),
    STXi(IndexMode),
    SUBAe(u16),
    CMPAe(u16),
    SBCAe(u16),
    SUBDe(u16),
    ANDAe(u16),
    BITAe(u16),
    LDAe(u16),
    STAe(u16),
    EORAe(u16),
    ADCAe(u16),
    ORAe(u16),
    ADDAe(u16),
    CMPXe(u16),
    JSRe(u16),
    LDXe(u16),
    STXe(u16),
    SUBBi8(u8),
    CMPBi8(u8),
    SBCBi8(u8),
    ADDDi16(u16),
    ANDBi8(u8),
    BITBi8(u8),
    LDBi8(u8),
    EORBi8(u8),
    ADCBi8(u8),
    ORBi8(u8),
    ADDBi8(u8),
    LDDi16(u16),
    LDQi32(u32),
    LDUi16(u16),
    SUBBd(u8),
    CMPBd(u8),
    SBCBd(u8),
    ADDDd(u8),
    ANDBd(u8),
    BITBd(u8),
    LDBd(u8),
    STBd(u8),
    EORBd(u8),
    ADCBd(u8),
    ORBd(u8),
    ADDBd(u8),
    LDDd(u8),
    STDd(u8),
    LDUd(u8),
    STUd(u8),
    SUBBi(IndexMode),
    CMPBi(IndexMode),
    SBCBi(IndexMode),
    ADDDi(IndexMode),
    ANDBi(IndexMode),
    BITBi(IndexMode),
    LDBi(IndexMode),
    STBi(IndexMode),
    EORBi(IndexMode),
    ADCBi(IndexMode),
    ORBi(IndexMode),
    ADDBi(IndexMode),
    LDDi(IndexMode),
    STDi(IndexMode),
    LDUi(IndexMode),
    STUi(IndexMode),
    SUBBe(u16),
    CMPBe(u16),
    SBCBe(u16),
    ADDDe(u16),
    ANDBe(u16),
    BITBe(u16),
    LDBe(u16),
    STBe(u16),
    EORBe(u16),
    ADCBe(u16),
    ORBe(u16),
    ADDBe(u16),
    LDDe(u16),
    STDe(u16),
    LDUe(u16),
    STUe(u16),
    LBRNlb(i16),
    LBlbc(Cond, i16),
    ADDRp(RegPair),
    ADCRp(RegPair),
    SUBRp(RegPair),
    SBCRp(RegPair),
    ANDRp(RegPair),
    ORRp(RegPair),
    EORRp(RegPair),
    CMPRp(RegPair),
    PSHSWx,
    PULSWx,
    PSHUWx,
    PULUWx,
    SWI2x,
    NEGDa,
    COMDa,
    LSRDa,
    RORDa,
    ASRDa,
    ASLDa,
    ROLDa,
    DECDa,
    INCDa,
    TSTDa,
    CLRDa,
    COMWa,
    LSRWa,
    RORWa,
    ROLWa,
    DECWa,
    INCWa,
    TSTWa,
    CLRWa,
    SUBWi16(u16),
    CMPWi16(u16),
    SBCDi16(u16),
    CMPDi16(u16),
    ANDDi16(u16),
    BITDi16(u16),
    LDWi16(u16),
    EORDi16(u16),
    ADCDi16(u16),
    ORDi16(u16),
    ADDWi16(u16),
    CMPYi16(u16),
    LDYi16(u16),
    SUBWd(u8),
    CMPWd(u8),
    SBCDd(u8),
    CMPDd(u8),
    ANDDd(u8),
    BITDd(u8),
    LDWd(u8),
    STWd(u8),
    EORDd(u8),
    ADCDd(u8),
    ORDd(u8),
    ADDWd(u8),
    CMPYd(u8),
    LDYd(u8),
    STYd(u8),
    SUBWi(IndexMode),
    CMPWi(IndexMode),
    SBCDi(IndexMode),
    CMPDi(IndexMode),
    ANDDi(IndexMode),
    BITDi(IndexMode),
    LDWi(IndexMode),
    STWi(IndexMode),
    EORDi(IndexMode),
    ADCDi(IndexMode),
    ORDi(IndexMode),
    ADDWi(IndexMode),
    CMPYi(IndexMode),
    LDYi(IndexMode),
    STYi(IndexMode),
    SUBWe(u16),
    CMPWe(u16),
    SBCDe(u16),
    CMPDe(u16),
    ANDDe(u16),
    BITDe(u16),
    LDWe(u16),
    STWe(u16),
    EORDe(u16),
    ADCDe(u16),
    ORDe(u16),
    ADDWe(u16),
    CMPYe(u16),
    LDYe(u16),
    STYe(u16),
    LDSi16(u16),
    LDQd(u8),
    STQd(u8),
    LDSd(u8),
    STSd(u8),
    LDQi(IndexMode),
    STQi(IndexMode),
    LDSi(IndexMode),
    STSi(IndexMode),
    LDQe(u16),
    STQe(u16),
    LDSe(u16),
    STSe(u16),
    BANDbd(BitSet, u8),
    BIANDbd(BitSet, u8),
    BORbd(BitSet, u8),
    BIORbd(BitSet, u8),
    BEORbd(BitSet, u8),
    BIEORbd(BitSet, u8),
    LDBTbd(BitSet, u8),
    STBTbd(BitSet, u8),
    TFM0pp(RegPair),
    TFM1pp(RegPair),
    TFM2pp(RegPair),
    TFM3pp(RegPair),
    BITMDi8(u8),
    LDMDi8(u8),
    SWI3x,
    COMEa,
    DECEa,
    INCEa,
    TSTEa,
    CLREa,
    COMFa,
    DECFa,
    INCFa,
    TSTFa,
    CLRFa,
    SUBEi8(u8),
    CMPEi8(u8),
    CMPUi16(u16),
    LDEi8(u8),
    ADDEi8(u8),
    CMPSi16(u16),
    DIVDi8(u8),
    DIVQi16(u16),
    MULDi16(u16),
    SUBEd(u8),
    CMPEd(u8),
    CMPUd(u8),
    LDEd(u8),
    STEd(u8),
    ADDEd(u8),
    CMPSd(u8),
    DIVDd(u8),
    DIVQd(u8),
    MULDd(u8),
    SUBEi(IndexMode),
    CMPEi(IndexMode),
    CMPUi(IndexMode),
    LDEi(IndexMode),
    STEi(IndexMode),
    ADDEi(IndexMode),
    CMPSi(IndexMode),
    DIVDi(IndexMode),
    DIVQi(IndexMode),
    MULDi(IndexMode),
    SUBEe(u16),
    CMPEe(u16),
    CMPUe(u16),
    LDEe(u16),
    STEe(u16),
    ADDEe(u16),
    CMPSe(u16),
    DIVDe(u16),
    DIVQe(u16),
    MULDe(u16),
    SUBFi8(u8),
    CMPFi8(u8),
    LDFi8(u8),
    ADDFi8(u8),
    SUBFd(u8),
    CMPFd(u8),
    LDFd(u8),
    STFd(u8),
    ADDFd(u8),
    SUBFi(IndexMode),
    CMPFi(IndexMode),
    LDFi(IndexMode),
    STFi(IndexMode),
    ADDFi(IndexMode),
    SUBFe(u16),
    CMPFe(u16),
    LDFe(u16),
    STFe(u16),
    ADDFe(u16),
    // MRVM END MARKER 1
}

#[derive(Debug)]
pub struct Processor {
    reg: Register,
    mem: Memory,
}

impl Processor {
    pub fn new() -> Processor {
        Processor {
            reg: Register::new(),
            mem: Memory::new(),
        }
    }

    fn continue_instruction_page_00(&mut self, opcode: u8) -> Instruction {
        match opcode {
            // MRVM START MARKER 2
            0x00 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::NEGd(addr8)
            }
            0x01 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::OIMid(imm8, addr8)
            }
            0x02 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::AIMid(imm8, addr8)
            }
            0x03 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::COMd(addr8)
            }
            0x04 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LSRd(addr8)
            }
            0x05 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::EIMid(imm8, addr8)
            }
            0x06 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::RORd(addr8)
            }
            0x07 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ASRd(addr8)
            }
            0x08 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ASLd(addr8)
            }
            0x09 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ROLd(addr8)
            }
            0x0A => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::DECd(addr8)
            }
            0x0B => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::TIMid(imm8, addr8)
            }
            0x0C => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::INCd(addr8)
            }
            0x0D => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::TSTd(addr8)
            }
            0x0E => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::JMPd(addr8)
            }
            0x0F => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CLRd(addr8)
            }
            0x12 => Instruction::NOPx,
            0x13 => Instruction::SYNCx,
            0x14 => Instruction::SEXWx,
            0x15 => Instruction::HCFx,
            0x16 => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBRAlb(pcoffset16)
            }
            0x17 => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBSRlb(pcoffset16)
            }
            0x19 => Instruction::DAAx,
            0x1A => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ORCCi8(imm8)
            }
            0x1C => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ANDCCi8(imm8)
            }
            0x1D => Instruction::SEXx,
            0x1E => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::EXGp(regpair)
            }
            0x1F => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::TFRp(regpair)
            }
            0x20 => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::BRAb(pcoffset8)
            }
            0x21 => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::BRNb(pcoffset8)
            }
            0x22 => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::HI, pcoffset8)
            }
            0x23 => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::LS, pcoffset8)
            }
            0x24 => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::HS, pcoffset8)
            }
            0x25 => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::LO, pcoffset8)
            }
            0x26 => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::NE, pcoffset8)
            }
            0x27 => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::EQ, pcoffset8)
            }
            0x28 => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::VC, pcoffset8)
            }
            0x29 => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::VS, pcoffset8)
            }
            0x2A => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::PL, pcoffset8)
            }
            0x2B => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::MI, pcoffset8)
            }
            0x2C => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::GE, pcoffset8)
            }
            0x2D => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::LT, pcoffset8)
            }
            0x2E => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::GT, pcoffset8)
            }
            0x2F => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::Bbc(Cond::LE, pcoffset8)
            }
            0x30 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LEAXi(index)
            }
            0x31 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LEAYi(index)
            }
            0x32 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LEASi(index)
            }
            0x33 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LEAUi(index)
            }
            0x34 => {
                let stackbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::PSHSs(stackbyte)
            }
            0x35 => {
                let stackbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::PULSs(stackbyte)
            }
            0x36 => {
                let stackbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::PSHUs(stackbyte)
            }
            0x37 => {
                let stackbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::PULUs(stackbyte)
            }
            0x39 => Instruction::RTSr,
            0x3A => Instruction::ABXx,
            0x3B => Instruction::RTIr,
            0x3C => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CWAIi8(imm8)
            }
            0x3D => Instruction::MULx,
            0x3E => Instruction::RESETx,
            0x3F => Instruction::SWIx,
            0x40 => Instruction::NEGAa,
            0x43 => Instruction::COMAa,
            0x44 => Instruction::LSRAa,
            0x46 => Instruction::RORAa,
            0x47 => Instruction::ASRAa,
            0x48 => Instruction::ASLAa,
            0x49 => Instruction::ROLAa,
            0x4A => Instruction::DECAa,
            0x4C => Instruction::INCAa,
            0x4D => Instruction::TSTAa,
            0x4F => Instruction::CLRAa,
            0x50 => Instruction::NEGBa,
            0x53 => Instruction::COMBa,
            0x54 => Instruction::LSRBa,
            0x56 => Instruction::RORBa,
            0x57 => Instruction::ASRBa,
            0x58 => Instruction::ASLBa,
            0x59 => Instruction::ROLBa,
            0x5A => Instruction::DECBa,
            0x5C => Instruction::INCBa,
            0x5D => Instruction::TSTBa,
            0x5F => Instruction::CLRBa,
            0x60 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::NEGi(index)
            }
            0x61 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::OIMii(imm8, index)
            }
            0x62 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::AIMii(imm8, index)
            }
            0x63 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::COMi(index)
            }
            0x64 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LSRi(index)
            }
            0x65 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::EIMii(imm8, index)
            }
            0x66 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::RORi(index)
            }
            0x67 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ASRi(index)
            }
            0x68 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ASLi(index)
            }
            0x69 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ROLi(index)
            }
            0x6A => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::DECi(index)
            }
            0x6B => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::TIMii(imm8, index)
            }
            0x6C => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::INCi(index)
            }
            0x6D => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::TSTi(index)
            }
            0x6E => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::JMPi(index)
            }
            0x6F => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::CLRi(index)
            }
            0x70 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::NEGe(addr16)
            }
            0x71 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::OIMie(imm8, addr16)
            }
            0x72 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::AIMie(imm8, addr16)
            }
            0x73 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::COMe(addr16)
            }
            0x74 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LSRe(addr16)
            }
            0x75 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::EIMie(imm8, addr16)
            }
            0x76 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::RORe(addr16)
            }
            0x77 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ASRe(addr16)
            }
            0x78 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ASLe(addr16)
            }
            0x79 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ROLe(addr16)
            }
            0x7A => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::DECe(addr16)
            }
            0x7B => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::TIMie(imm8, addr16)
            }
            0x7C => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::INCe(addr16)
            }
            0x7D => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::TSTe(addr16)
            }
            0x7E => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::JMPe(addr16)
            }
            0x7F => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CLRe(addr16)
            }
            0x80 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SUBAi8(imm8)
            }
            0x81 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPAi8(imm8)
            }
            0x82 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SBCAi8(imm8)
            }
            0x83 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SUBDi16(imm16)
            }
            0x84 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ANDAi8(imm8)
            }
            0x85 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BITAi8(imm8)
            }
            0x86 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDAi8(imm8)
            }
            0x88 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::EORAi8(imm8)
            }
            0x89 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADCAi8(imm8)
            }
            0x8A => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ORAi8(imm8)
            }
            0x8B => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADDAi8(imm8)
            }
            0x8C => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPXi16(imm16)
            }
            0x8D => {
                let pcoffset8 = self.mem.read8(&self.reg.pc) as i8;
                self.reg.pc_increment();
                Instruction::BSRb(pcoffset8)
            }
            0x8E => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDXi16(imm16)
            }
            0x90 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SUBAd(addr8)
            }
            0x91 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPAd(addr8)
            }
            0x92 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SBCAd(addr8)
            }
            0x93 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SUBDd(addr8)
            }
            0x94 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ANDAd(addr8)
            }
            0x95 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BITAd(addr8)
            }
            0x96 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDAd(addr8)
            }
            0x97 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STAd(addr8)
            }
            0x98 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::EORAd(addr8)
            }
            0x99 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADCAd(addr8)
            }
            0x9A => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ORAd(addr8)
            }
            0x9B => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADDAd(addr8)
            }
            0x9C => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPXd(addr8)
            }
            0x9D => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::JSRd(addr8)
            }
            0x9E => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDXd(addr8)
            }
            0x9F => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STXd(addr8)
            }
            0xA0 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::SUBAi(index)
            }
            0xA1 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::CMPAi(index)
            }
            0xA2 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::SBCAi(index)
            }
            0xA3 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::SUBDi(index)
            }
            0xA4 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ANDAi(index)
            }
            0xA5 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::BITAi(index)
            }
            0xA6 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LDAi(index)
            }
            0xA7 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::STAi(index)
            }
            0xA8 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::EORAi(index)
            }
            0xA9 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ADCAi(index)
            }
            0xAA => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ORAi(index)
            }
            0xAB => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ADDAi(index)
            }
            0xAC => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::CMPXi(index)
            }
            0xAD => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::JSRi(index)
            }
            0xAE => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LDXi(index)
            }
            0xAF => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::STXi(index)
            }
            0xB0 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SUBAe(addr16)
            }
            0xB1 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPAe(addr16)
            }
            0xB2 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SBCAe(addr16)
            }
            0xB3 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SUBDe(addr16)
            }
            0xB4 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ANDAe(addr16)
            }
            0xB5 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::BITAe(addr16)
            }
            0xB6 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDAe(addr16)
            }
            0xB7 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::STAe(addr16)
            }
            0xB8 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::EORAe(addr16)
            }
            0xB9 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADCAe(addr16)
            }
            0xBA => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ORAe(addr16)
            }
            0xBB => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADDAe(addr16)
            }
            0xBC => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPXe(addr16)
            }
            0xBD => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::JSRe(addr16)
            }
            0xBE => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDXe(addr16)
            }
            0xBF => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::STXe(addr16)
            }
            0xC0 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SUBBi8(imm8)
            }
            0xC1 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPBi8(imm8)
            }
            0xC2 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SBCBi8(imm8)
            }
            0xC3 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADDDi16(imm16)
            }
            0xC4 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ANDBi8(imm8)
            }
            0xC5 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BITBi8(imm8)
            }
            0xC6 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDBi8(imm8)
            }
            0xC8 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::EORBi8(imm8)
            }
            0xC9 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADCBi8(imm8)
            }
            0xCA => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ORBi8(imm8)
            }
            0xCB => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADDBi8(imm8)
            }
            0xCC => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDDi16(imm16)
            }
            0xCD => {
                let imm32 = self.mem.read32(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDQi32(imm32)
            }
            0xCE => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDUi16(imm16)
            }
            0xD0 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SUBBd(addr8)
            }
            0xD1 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPBd(addr8)
            }
            0xD2 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SBCBd(addr8)
            }
            0xD3 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADDDd(addr8)
            }
            0xD4 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ANDBd(addr8)
            }
            0xD5 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BITBd(addr8)
            }
            0xD6 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDBd(addr8)
            }
            0xD7 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STBd(addr8)
            }
            0xD8 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::EORBd(addr8)
            }
            0xD9 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADCBd(addr8)
            }
            0xDA => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ORBd(addr8)
            }
            0xDB => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADDBd(addr8)
            }
            0xDC => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDDd(addr8)
            }
            0xDD => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STDd(addr8)
            }
            0xDE => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDUd(addr8)
            }
            0xDF => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STUd(addr8)
            }
            0xE0 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::SUBBi(index)
            }
            0xE1 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::CMPBi(index)
            }
            0xE2 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::SBCBi(index)
            }
            0xE3 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ADDDi(index)
            }
            0xE4 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ANDBi(index)
            }
            0xE5 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::BITBi(index)
            }
            0xE6 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LDBi(index)
            }
            0xE7 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::STBi(index)
            }
            0xE8 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::EORBi(index)
            }
            0xE9 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ADCBi(index)
            }
            0xEA => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ORBi(index)
            }
            0xEB => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ADDBi(index)
            }
            0xEC => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LDDi(index)
            }
            0xED => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::STDi(index)
            }
            0xEE => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LDUi(index)
            }
            0xEF => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::STUi(index)
            }
            0xF0 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SUBBe(addr16)
            }
            0xF1 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPBe(addr16)
            }
            0xF2 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SBCBe(addr16)
            }
            0xF3 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADDDe(addr16)
            }
            0xF4 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ANDBe(addr16)
            }
            0xF5 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::BITBe(addr16)
            }
            0xF6 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDBe(addr16)
            }
            0xF7 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::STBe(addr16)
            }
            0xF8 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::EORBe(addr16)
            }
            0xF9 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADCBe(addr16)
            }
            0xFA => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ORBe(addr16)
            }
            0xFB => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADDBe(addr16)
            }
            0xFC => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDDe(addr16)
            }
            0xFD => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::STDe(addr16)
            }
            0xFE => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDUe(addr16)
            }
            0xFF => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::STUe(addr16)
            }
            // MRVM END MARKER 2
            _ => Instruction::HCFx,
        }
    }

    fn continue_instruction_page_10(&mut self, opcode: u8) -> Instruction {
        match opcode {
            // MRVM START MARKER 3
            0x21 => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBRNlb(pcoffset16)
            }
            0x22 => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::HI, pcoffset16)
            }
            0x23 => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::LS, pcoffset16)
            }
            0x24 => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::HS, pcoffset16)
            }
            0x25 => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::LO, pcoffset16)
            }
            0x26 => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::NE, pcoffset16)
            }
            0x27 => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::EQ, pcoffset16)
            }
            0x28 => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::VC, pcoffset16)
            }
            0x29 => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::VS, pcoffset16)
            }
            0x2A => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::PL, pcoffset16)
            }
            0x2B => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::MI, pcoffset16)
            }
            0x2C => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::GE, pcoffset16)
            }
            0x2D => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::LT, pcoffset16)
            }
            0x2E => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::GT, pcoffset16)
            }
            0x2F => {
                let pcoffset16 = self.mem.read16(&self.reg.pc) as i16;
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LBlbc(Cond::LE, pcoffset16)
            }
            0x30 => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::ADDRp(regpair)
            }
            0x31 => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::ADCRp(regpair)
            }
            0x32 => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::SUBRp(regpair)
            }
            0x33 => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::SBCRp(regpair)
            }
            0x34 => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::ANDRp(regpair)
            }
            0x35 => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::ORRp(regpair)
            }
            0x36 => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::EORRp(regpair)
            }
            0x37 => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::CMPRp(regpair)
            }
            0x38 => Instruction::PSHSWx,
            0x39 => Instruction::PULSWx,
            0x3A => Instruction::PSHUWx,
            0x3B => Instruction::PULUWx,
            0x3F => Instruction::SWI2x,
            0x40 => Instruction::NEGDa,
            0x43 => Instruction::COMDa,
            0x44 => Instruction::LSRDa,
            0x46 => Instruction::RORDa,
            0x47 => Instruction::ASRDa,
            0x48 => Instruction::ASLDa,
            0x49 => Instruction::ROLDa,
            0x4A => Instruction::DECDa,
            0x4C => Instruction::INCDa,
            0x4D => Instruction::TSTDa,
            0x4F => Instruction::CLRDa,
            0x53 => Instruction::COMWa,
            0x54 => Instruction::LSRWa,
            0x56 => Instruction::RORWa,
            0x59 => Instruction::ROLWa,
            0x5A => Instruction::DECWa,
            0x5C => Instruction::INCWa,
            0x5D => Instruction::TSTWa,
            0x5F => Instruction::CLRWa,
            0x80 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SUBWi16(imm16)
            }
            0x81 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPWi16(imm16)
            }
            0x82 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SBCDi16(imm16)
            }
            0x83 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPDi16(imm16)
            }
            0x84 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ANDDi16(imm16)
            }
            0x85 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::BITDi16(imm16)
            }
            0x86 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDWi16(imm16)
            }
            0x88 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::EORDi16(imm16)
            }
            0x89 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADCDi16(imm16)
            }
            0x8A => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ORDi16(imm16)
            }
            0x8B => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADDWi16(imm16)
            }
            0x8C => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPYi16(imm16)
            }
            0x8E => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDYi16(imm16)
            }
            0x90 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SUBWd(addr8)
            }
            0x91 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPWd(addr8)
            }
            0x92 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SBCDd(addr8)
            }
            0x93 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPDd(addr8)
            }
            0x94 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ANDDd(addr8)
            }
            0x95 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BITDd(addr8)
            }
            0x96 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDWd(addr8)
            }
            0x97 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STWd(addr8)
            }
            0x98 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::EORDd(addr8)
            }
            0x99 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADCDd(addr8)
            }
            0x9A => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ORDd(addr8)
            }
            0x9B => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADDWd(addr8)
            }
            0x9C => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPYd(addr8)
            }
            0x9E => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDYd(addr8)
            }
            0x9F => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STYd(addr8)
            }
            0xA0 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::SUBWi(index)
            }
            0xA1 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::CMPWi(index)
            }
            0xA2 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::SBCDi(index)
            }
            0xA3 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::CMPDi(index)
            }
            0xA4 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ANDDi(index)
            }
            0xA5 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::BITDi(index)
            }
            0xA6 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LDWi(index)
            }
            0xA7 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::STWi(index)
            }
            0xA8 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::EORDi(index)
            }
            0xA9 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ADCDi(index)
            }
            0xAA => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ORDi(index)
            }
            0xAB => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ADDWi(index)
            }
            0xAC => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::CMPYi(index)
            }
            0xAE => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LDYi(index)
            }
            0xAF => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::STYi(index)
            }
            0xB0 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SUBWe(addr16)
            }
            0xB1 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPWe(addr16)
            }
            0xB2 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SBCDe(addr16)
            }
            0xB3 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPDe(addr16)
            }
            0xB4 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ANDDe(addr16)
            }
            0xB5 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::BITDe(addr16)
            }
            0xB6 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDWe(addr16)
            }
            0xB7 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::STWe(addr16)
            }
            0xB8 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::EORDe(addr16)
            }
            0xB9 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADCDe(addr16)
            }
            0xBA => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ORDe(addr16)
            }
            0xBB => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADDWe(addr16)
            }
            0xBC => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPYe(addr16)
            }
            0xBE => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDYe(addr16)
            }
            0xBF => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::STYe(addr16)
            }
            0xCE => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDSi16(imm16)
            }
            0xDC => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDQd(addr8)
            }
            0xDD => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STQd(addr8)
            }
            0xDE => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDSd(addr8)
            }
            0xDF => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STSd(addr8)
            }
            0xEC => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LDQi(index)
            }
            0xED => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::STQi(index)
            }
            0xEE => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LDSi(index)
            }
            0xEF => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::STSi(index)
            }
            0xFC => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDQe(addr16)
            }
            0xFD => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::STQe(addr16)
            }
            0xFE => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDSe(addr16)
            }
            0xFF => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::STSe(addr16)
            }
            // MRVM END MARKER 3
            _ => Instruction::HCFx,
        }
    }

    fn continue_instruction_page_11(&mut self, opcode: u8) -> Instruction {
        match opcode {
            // MRVM START MARKER 4
            0x30 => {
                let bitset = BitSet::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BANDbd(bitset, addr8)
            }
            0x31 => {
                let bitset = BitSet::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BIANDbd(bitset, addr8)
            }
            0x32 => {
                let bitset = BitSet::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BORbd(bitset, addr8)
            }
            0x33 => {
                let bitset = BitSet::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BIORbd(bitset, addr8)
            }
            0x34 => {
                let bitset = BitSet::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BEORbd(bitset, addr8)
            }
            0x35 => {
                let bitset = BitSet::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BIEORbd(bitset, addr8)
            }
            0x36 => {
                let bitset = BitSet::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDBTbd(bitset, addr8)
            }
            0x37 => {
                let bitset = BitSet::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STBTbd(bitset, addr8)
            }
            0x38 => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::TFM0pp(regpair)
            }
            0x39 => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::TFM1pp(regpair)
            }
            0x3A => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::TFM2pp(regpair)
            }
            0x3B => {
                let regpair = RegPair::new(self.mem.read8(&self.reg.pc));
                self.reg.pc_increment();
                Instruction::TFM3pp(regpair)
            }
            0x3C => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::BITMDi8(imm8)
            }
            0x3D => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDMDi8(imm8)
            }
            0x3F => Instruction::SWI3x,
            0x43 => Instruction::COMEa,
            0x4A => Instruction::DECEa,
            0x4C => Instruction::INCEa,
            0x4D => Instruction::TSTEa,
            0x4F => Instruction::CLREa,
            0x53 => Instruction::COMFa,
            0x5A => Instruction::DECFa,
            0x5C => Instruction::INCFa,
            0x5D => Instruction::TSTFa,
            0x5F => Instruction::CLRFa,
            0x80 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SUBEi8(imm8)
            }
            0x81 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPEi8(imm8)
            }
            0x83 => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPUi16(imm16)
            }
            0x86 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDEi8(imm8)
            }
            0x8B => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADDEi8(imm8)
            }
            0x8C => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPSi16(imm16)
            }
            0x8D => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::DIVDi8(imm8)
            }
            0x8E => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::DIVQi16(imm16)
            }
            0x8F => {
                let imm16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::MULDi16(imm16)
            }
            0x90 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SUBEd(addr8)
            }
            0x91 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPEd(addr8)
            }
            0x93 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPUd(addr8)
            }
            0x96 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDEd(addr8)
            }
            0x97 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STEd(addr8)
            }
            0x9B => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADDEd(addr8)
            }
            0x9C => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPSd(addr8)
            }
            0x9D => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::DIVDd(addr8)
            }
            0x9E => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::DIVQd(addr8)
            }
            0x9F => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::MULDd(addr8)
            }
            0xA0 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::SUBEi(index)
            }
            0xA1 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::CMPEi(index)
            }
            0xA3 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::CMPUi(index)
            }
            0xA6 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LDEi(index)
            }
            0xA7 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::STEi(index)
            }
            0xAB => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ADDEi(index)
            }
            0xAC => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::CMPSi(index)
            }
            0xAD => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::DIVDi(index)
            }
            0xAE => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::DIVQi(index)
            }
            0xAF => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::MULDi(index)
            }
            0xB0 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SUBEe(addr16)
            }
            0xB1 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPEe(addr16)
            }
            0xB3 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPUe(addr16)
            }
            0xB6 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDEe(addr16)
            }
            0xB7 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::STEe(addr16)
            }
            0xBB => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADDEe(addr16)
            }
            0xBC => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPSe(addr16)
            }
            0xBD => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::DIVDe(addr16)
            }
            0xBE => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::DIVQe(addr16)
            }
            0xBF => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::MULDe(addr16)
            }
            0xC0 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SUBFi8(imm8)
            }
            0xC1 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPFi8(imm8)
            }
            0xC6 => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDFi8(imm8)
            }
            0xCB => {
                let imm8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADDFi8(imm8)
            }
            0xD0 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::SUBFd(addr8)
            }
            0xD1 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::CMPFd(addr8)
            }
            0xD6 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::LDFd(addr8)
            }
            0xD7 => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::STFd(addr8)
            }
            0xDB => {
                let addr8 = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                Instruction::ADDFd(addr8)
            }
            0xE0 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::SUBFi(index)
            }
            0xE1 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::CMPFi(index)
            }
            0xE6 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::LDFi(index)
            }
            0xE7 => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::STFi(index)
            }
            0xEB => {
                let indexbyte = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                let byte1 = self.mem.read8(&(self.reg.pc + 1));
                let byte2 = self.mem.read8(&(self.reg.pc + 2));
                let index = IndexMode::new(indexbyte, byte1, byte2);
                Instruction::ADDFi(index)
            }
            0xF0 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::SUBFe(addr16)
            }
            0xF1 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::CMPFe(addr16)
            }
            0xF6 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::LDFe(addr16)
            }
            0xF7 => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::STFe(addr16)
            }
            0xFB => {
                let addr16 = self.mem.read16(&self.reg.pc);
                self.reg.pc_increment();
                self.reg.pc_increment();
                Instruction::ADDFe(addr16)
            }
            // MRVM END MARKER 4
            _ => Instruction::HCFx,
        }
    }

    fn decode_next_instruction(&mut self) -> Instruction {
        let mut opcode = self.mem.read8(&self.reg.pc);
        self.reg.pc_increment();
        match opcode {
            0x10 => {
                opcode = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                self.continue_instruction_page_10(opcode)
            }
            0x11 => {
                opcode = self.mem.read8(&self.reg.pc);
                self.reg.pc_increment();
                self.continue_instruction_page_11(opcode)
            }
            _ => self.continue_instruction_page_00(opcode),
        }
    }

    pub fn execute_instruction(&mut self) {
        let opcode = self.decode_next_instruction();
    }

    pub fn set_a(&mut self, value: &u8) {
        self.reg.a = *value;
    }

    pub fn get_a(&self) -> u8 {
        self.reg.a
    }

    pub fn set_b(&mut self, value: &u8) {
        self.reg.b = *value;
    }

    pub fn get_b(&self) -> u8 {
        self.reg.b
    }

    pub fn set_d(&mut self, value: &u16) {
        let bytes = value.to_be_bytes();
        self.reg.a = bytes[0];
        self.reg.b = bytes[1];
    }

    pub fn get_d(&self) -> u16 {
        u16::from_be_bytes([self.reg.a, self.reg.b])
    }

    pub fn set_dp(&mut self, value: &u8) {
        self.reg.dp = *value;
    }

    pub fn get_dp(&self) -> u8 {
        self.reg.dp
    }

    pub fn set_cc(&mut self, value: &u8) {
        self.reg.cc = *value;
    }

    pub fn get_cc(&self) -> u8 {
        self.reg.cc
    }

    pub fn set_x(&mut self, value: &u16) {
        self.reg.x = *value;
    }

    pub fn get_x(&self) -> u16 {
        self.reg.x
    }

    pub fn set_y(&mut self, value: &u16) {
        self.reg.y = *value;
    }

    pub fn get_y(&self) -> u16 {
        self.reg.y
    }

    pub fn set_u(&mut self, value: &u16) {
        self.reg.u = *value;
    }

    pub fn get_u(&self) -> u16 {
        self.reg.u
    }

    pub fn set_s(&mut self, value: &u16) {
        self.reg.s = *value;
    }

    pub fn get_s(&self) -> u16 {
        self.reg.s
    }

    pub fn set_pc(&mut self, value: &u16) {
        self.reg.x = *value;
    }

    pub fn get_pc(&self) -> u16 {
        self.reg.pc
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use std::num::Wrapping;

    #[test]
    fn test_full_memmap_read_write() {
        let mut memory = Memory::new();
        let mut byte = Wrapping(13u8);
        let mut val: u8;
        for address in 0u16..=u16::MAX {
            val = memory.read8(&address);
            println!("{address:04X} {val:02X}");
            assert_eq!(memory.read8(&address), 0);
            memory.write(&address, byte.0);
            if address < 56 * 1024u16 {
                assert_eq!(memory.read8(&address), byte.0);
            } else if address < 56 * 1024u16 + 512 {
                assert_eq!(memory.read8(&address), 0b00000000u8);
            } else {
                assert_eq!(memory.read8(&address), 0);
            }
            byte += 1;
        }
        byte = Wrapping(13u8);
        for address in 0u16..=u16::MAX {
            if address < 56 * 1024u16 {
                assert_eq!(memory.read8(&address), byte.0);
                memory.write(&address, 0);
                assert_eq!(memory.read8(&address), 0);
            } else if address < 56 * 1024u16 + 512 {
                assert_eq!(memory.read8(&address), 0);
                memory.write(&address, 0);
                assert_eq!(memory.read8(&address), 0);
            } else {
                assert_eq!(memory.read8(&address), 0);
            }
            byte += 1;
        }
    }

    #[test]
    fn test_register_abd() {
        let mut processor = Processor::new();
        assert_eq!(processor.get_d(), 0x0000u16);
        assert_eq!(processor.get_a(), 0x00u8);
        assert_eq!(processor.get_b(), 0x00u8);
        processor.set_d(&0x1234u16);
        assert_eq!(processor.get_d(), 0x1234u16);
        assert_eq!(processor.get_a(), 0x12u8);
        assert_eq!(processor.get_b(), 0x34u8);
        processor.set_d(&0xFFFFu16);
        assert_eq!(processor.get_d(), 0xFFFFu16);
        assert_eq!(processor.get_a(), 0xFFu8);
        assert_eq!(processor.get_b(), 0xFFu8);
        processor.set_a(&0x00u8);
        assert_eq!(processor.get_d(), 0x00FFu16);
        assert_eq!(processor.get_a(), 0x00u8);
        assert_eq!(processor.get_b(), 0xFFu8);
        processor.set_a(&0xFFu8);
        assert_eq!(processor.get_d(), 0xFFFFu16);
        assert_eq!(processor.get_a(), 0xFFu8);
        assert_eq!(processor.get_b(), 0xFFu8);
        processor.set_b(&0x00u8);
        assert_eq!(processor.get_d(), 0xFF00u16);
        assert_eq!(processor.get_a(), 0xFFu8);
        assert_eq!(processor.get_b(), 0x00u8);
        processor.set_b(&0xFFu8);
        assert_eq!(processor.get_d(), 0xFFFFu16);
        assert_eq!(processor.get_a(), 0xFFu8);
        assert_eq!(processor.get_b(), 0xFFu8);
        processor.set_a(&0x56u8);
        processor.set_b(&0x78u8);
        assert_eq!(processor.get_d(), 0x5678u16);
        assert_eq!(processor.get_a(), 0x56u8);
        assert_eq!(processor.get_b(), 0x78u8);
        processor.set_a(&0x00u8);
        processor.set_b(&0x00u8);
        assert_eq!(processor.get_d(), 0x0000u16);
        assert_eq!(processor.get_a(), 0x00u8);
        assert_eq!(processor.get_b(), 0x00u8);
    }

    #[test]
    fn test_register_x() {
        let mut processor = Processor::new();
        assert_eq!(processor.get_x(), 0x0000u16);
        processor.set_x(&0x1234u16);
        assert_eq!(processor.get_x(), 0x1234u16);
        processor.set_x(&0x0000u16);
        assert_eq!(processor.get_x(), 0x0000u16);
    }
}
