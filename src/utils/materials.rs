use bytemuck::{Pod, Zeroable};

use palette::encoding::Linear;
use palette::rgb::Rgb;
use palette::Srgb;
use phf::phf_map;

pub const PIPE_TY_MIN: i32 = 20;
pub const PIPE_TY_MAX: i32 = 40;
pub const EQ_TY_MIN: i32 = 80;
pub const EQ_TY_MAX: i32 = 100;

pub const TY_HULL_PROFILES: i32 = 74;
pub const TY_HULL_PLATES: i32 = 84;
pub const TY_HULL_OUTERPLATES: i32 = 86;
pub const TY_HULL_OTHERS: i32 = 15;
pub const MATERIALS_COUNT: usize = 140;
pub const SELECTION_HULL_MAT: i32 = 1;
pub const HIDDEN_HULL_MAT: i32 = 0;

pub enum HullPartTypes {
    _ShellLongitudinal = 0,
    Decklongitudinal = 1,
    _Hullframe = 2,
    Deckbeam = 3,
    Stifflongbulkhead = 4,
    Stifftransbulkhead = 5,
    Generalprofile = 6,
    _Foundationprofil = 7,
    Internalplate = 8,
    _Shellplate = 9,
    Deckplate = 10,
    Templateplate = 11,
    _Foundationplate = 12,
    Shellmarkingplate = 13,
    Corrugatedplate = 14,
    _InternalPlateFHull = 15,
    _InternalProfileFHull = 16,
    _InternalLCFHull = 17,
    _InternalStandardPlateFHull = 18,
    _InternalStraightProfileFHull = 19,
    _InternalCollarPlateFHull = 20,
    _InternalGridProfile = 21,
    _InternalBracketFHull = 22,
    _InternalMacroPlateFHull = 23,
    _InternalCurvedProfileFHull = 24,
    InternalCorrugatedStandardPlate = 25,
    InternalCorrugatedMacroPlate = 26,
    DevelopedShellorDeckPlate = 31,
    ShellorDeckBendingTemplate = 32,
    JigBasePart = 33,
    Straightweb = 34,
    Straightflange = 35,
    Shapedweb = 36,
    Shapedflange = 37,
    Importedpart = 38,
}

pub static OPENCOLORS: phf::Map<&'static str, &'static [u32; 10]> = phf_map! {
"gray"=>&[0xf8f9fa,0xf1f3f5,0xe9ecef,0xdee2e6,0xced4da,0xadb5bd,0x868e96,0x495057,0x343a40,0x212529],
"red"=>&[0xfff5f5,0xffe3e3,0xffc9c9,0xffa8a8,0xff8787,0xff6b6b,0xfa5252,0xf03e3e,0xe03131,0xc92a2a],
"pink"=>&[0xfff0f6,0xffdeeb,0xfcc2d7,0xfaa2c1,0xf783ac,0xf06595,0xe64980,0xd6336c,0xc2255c,0xa61e4d],
"grape"=>&[0xf8f0fc,0xf3d9fa,0xeebefa,0xe599f7,0xda77f2,0xcc5de8,0xbe4bdb,0xae3ec9,0x9c36b5,0x862e9c],
"violet"=>&[0xf3f0ff,0xe5dbff,0xd0bfff,0xb197fc,0x9775fa,0x845ef7,0x7950f2,0x7048e8,0x6741d9,0x5f3dc4],
"indigo"=>&[0xedf2ff,0xdbe4ff,0xbac8ff,0x91a7ff,0x748ffc,0x5c7cfa,0x4c6ef5,0x4263eb,0x3b5bdb,0x364fc7],
"blue"=>&[0xe7f5ff,0xd0ebff,0xa5d8ff,0x74c0fc,0x4dabf7,0x339af0,0x228be6,0x1c7ed6,0x1971c2,0x1864ab],
"cyan"=>&[0xe3fafc,0xc5f6fa,0x99e9f2,0x66d9e8,0x3bc9db,0x22b8cf,0x15aabf,0x1098ad,0x0c8599,0x0b7285],
"teal"=>&[0xe6fcf5,0xc3fae8,0x96f2d7,0x63e6be,0x38d9a9,0x20c997,0x12b886,0x0ca678,0x099268,0x087f5b],
"green"=>&[0xebfbee,0xd3f9d8,0xb2f2bb,0x8ce99a,0x69db7c,0x51cf66,0x40c057,0x37b24d,0x2f9e44,0x2b8a3e],
"lime"=>&[0xf4fce3,0xe9fac8,0xd8f5a2,0xc0eb75,0xa9e34b,0x94d82d,0x82c91e,0x74b816,0x66a80f,0x5c940d],
"yellow"=>&[0xfff9db,0xfff3bf,0xffec99,0xffe066,0xffd43b,0xfcc419,0xfab005,0xf59f00,0xf08c00,0xe67700],
"orange"=>&[0xfff4e6,0xffe8cc,0xffd8a8,0xffc078,0xffa94d,0xff922b,0xfd7e14,0xf76707,0xe8590c,0xd9480f]
};

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct Material {
    pub color: [f32; 4],
    pub specular_color: [f32; 4],
    pub ambient_intensity: f32,
    pub diffuse_intensity: f32,
    pub specular_intensity: f32,
    pub specular_shininess: f32,
}

impl Material {
    fn default(c: [f32; 4]) -> Self {
        Self::new(
            c,
            [1.0, 1.0, 1.0],
            0.7,
            1.0,
            0.05,
            8.0,
        )
    }
    fn new(c: [f32; 4], sc: [f32; 3], ai: f32, di: f32, si: f32, ss: f32) -> Self {
        Self {
            color: [c[0], c[1], c[2], c[3]],
            specular_color: [sc[0], sc[1], sc[2], 1.0],
            ambient_intensity: ai,
            diffuse_intensity: di,
            specular_intensity: si,
            specular_shininess: ss,
        }
    }
    pub fn generate_materials() -> Vec<Material> {
        let mut ret: Vec<Material> = vec![];
        let alfa: f32 = 1.0;
        ret.push(Material::default([0.0, 0.0, 0.0, 0.0]));// HIDDEN
        ret.push(Material::default([1.0, 1.0, 1.0, 1.0]));// SELECT
        ret.push(Material::default([0.0, 0.0, 0.0, 1.0]));
        ret.push(Material::default([0.0, 0.0, 0.0, 1.0]));
        ret.push(Material::default([0.0, 0.0, 0.0, 1.0]));
        ret.push(Material::default([0.0, 0.0, 0.0, 1.0]));
        ret.push(Material::default([0.0, 0.0, 0.0, 1.0]));
        ret.push(Material::default([0.0, 0.0, 0.0, 1.0]));
        ret.push(Material::default([0.0, 0.0, 0.0, 1.0]));
        ret.push(Material::default([0.0, 0.0, 0.0, 1.0]));
        OPENCOLORS.values().for_each(|&color_group| {
            color_group.iter().for_each(|c| {
                let srgb = Srgb::from(*c).into_linear() as Rgb<Linear<palette::encoding::Srgb>, f32>;
                let mm = Material::default([srgb.red, srgb.green, srgb.blue, alfa]);
                ret.push(mm);
            })
        });
        ret
    }
    pub fn type_to_color(ty: i32) -> i32 {
        match ty {
            0 | 2 | 7 | 16 | 19 | 21 | 24 | 17 => TY_HULL_PROFILES, //PROFILES
            8 | 12 | 15 | 18 | 20 | 22 | 23 => TY_HULL_PLATES,//PLATES
            9 => TY_HULL_OUTERPLATES,//HULL PLATES
            _ => TY_HULL_OTHERS
        }
    }
}
