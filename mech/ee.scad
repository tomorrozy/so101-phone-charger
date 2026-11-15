// 手机无线充电机械臂 · 末端参数化模型
// 单位 mm。没有实物，腕接口是占位值，量完改前面 6 个参数即可。
// 打印件只有两个：plate（腕接板+相机架）和 puck（盘体）。

$fn = 96;

PART = "assembly";   // "plate" | "puck" | "foam" | "assembly"

// ================= 腕部接口（占位，等实测） =================
wrist_pitch_x   = 40;    // 4 个 M3 孔的横向中心距
wrist_pitch_y   = 40;    // 纵向中心距
wrist_hole_d    = 3.2;   // M3 过孔
wrist_head_d    = 6.0;   // M3 沉头
wrist_head_dep  = 3.0;
horn_d          = 26;    // 舵机盘直径（占位）
horn_h          = 4;     // 舵机盘凸出腕板的高度（占位）

// ================= 腕接板 =================
plate_od  = 64;
plate_t   = 6;

// ================= 浮动 =================
float_gap      = 6;
float_screw_r  = 26.2;
float_angles   = [0, 120, 240];
shoulder_d     = 5.6;    // M2.5 肩螺丝头
shoulder_dep   = 2.6;
spring_d       = 8.0;
spring_seat    = 1.2;

// ================= 盘体 =================
puck_od            = 60;
puck_h             = 12;
face_wall          = 3;
coil_pocket_d      = 42.4;
coil_pocket_depth  = 9;    // 前面留 3mm 壁
magnet_d           = 5.0;  // N35 Ø5×3，4 颗分布成环
magnet_pocket_d    = 5.3;
magnet_h           = 3.0;
magnet_r           = 25.5;
magnet_angles      = [45, 135, 225, 315];
insert_d           = 3.8;  // M2.5 热熔螺母
insert_depth       = 5;
foam_d             = 52;
foam_t             = 3;

// ================= 相机 =================
cam_offset   = 48;
cam_lens_z   = 12;
cam_board_w  = 36;
cam_board_h  = 40;
cam_board_t  = 4;
cam_module   = 33.0;   // 32.4 模块 + 0.3/边间隙
cam_recess   = 1.5;

// ================= 推导 =================
puck_z0      = plate_t + float_gap;         // 12
puck_face_z  = puck_z0 + puck_h;            // 24
hover_mm     = 60;
cam_tilt     = atan2(cam_offset, puck_face_z + hover_mm - cam_lens_z);

echo(str("cam_tilt = ", cam_tilt));

// ================= 零件 =================
module cam_board_geo() {
    translate([cam_offset, 0, cam_lens_z]) rotate([0, -cam_tilt, 0]) {
        translate([-cam_board_w/2, -cam_board_h/2, -cam_board_t/2])
            cube([cam_board_w, cam_board_h, cam_board_t]);
    }
}

module plate() {
    difference() {
        union() {
            cylinder(d = plate_od, h = plate_t);
            // 侧向悬臂，延伸到相机板下方
            translate([26, -13, 0]) cube([16, 26, plate_t]);
            // 相机板
            cam_board_geo();
        }

        // 腕部 4 个 M3
        for (sx = [-1, 1], sy = [-1, 1])
            translate([sx * wrist_pitch_x / 2, sy * wrist_pitch_y / 2, -1]) {
                cylinder(d = wrist_hole_d, h = plate_t + 2);
                translate([0, 0, plate_t - wrist_head_dep])
                    cylinder(d = wrist_head_d + 0.4, h = wrist_head_dep + 1);
            }

        // 舵机盘让位
        translate([0, 0, -1]) cylinder(d = horn_d + 1, h = horn_h + 1);

        // 浮动螺钉孔 + 沉孔 + 弹簧座
        for (a = float_angles) rotate([0, 0, a]) translate([float_screw_r, 0, 0]) {
            translate([0, 0, -1]) cylinder(d = 2.9, h = plate_t + 2);
            translate([0, 0, plate_t - shoulder_dep])
                cylinder(d = shoulder_d, h = shoulder_dep + 1);
            translate([0, 0, plate_t - spring_seat])
                cylinder(d = spring_d, h = spring_seat + 1);
        }

        // 出线槽（正面，走底部）
        translate([-3, -plate_od / 2, plate_t - 4]) cube([6, 17, 5]);

        // 相机模块浅槽（正面）
        translate([cam_offset, 0, cam_lens_z]) rotate([0, -cam_tilt, 0])
            translate([-cam_module / 2, -cam_module / 2, cam_board_t / 2 - cam_recess])
                cube([cam_module, cam_module, cam_recess + 0.1]);

    }
}

module puck() {
    translate([0, 0, puck_z0])
        difference() {
            union() {
                cylinder(d = puck_od, h = puck_h);
                // 防坠挂绳耳
                translate([0, -puck_od / 2 - 0.5, 2]) cube([10, 6, 4], center = true);
            }

            // 线圈仓（从背面开）
            translate([0, 0, -0.1]) cylinder(d = coil_pocket_d, h = coil_pocket_depth + 0.1);

            // 磁铁：从背面开孔，磁铁压到离盘面 3mm
            for (a = magnet_angles) rotate([0, 0, a]) translate([magnet_r, 0, 0])
                translate([0, 0, -0.1])
                    cylinder(d = magnet_pocket_d, h = puck_h - face_wall - 0.1);

            // 热熔螺母
            for (a = float_angles) rotate([0, 0, a]) translate([float_screw_r, 0, 0])
                translate([0, 0, -0.1]) cylinder(d = insert_d, h = insert_depth + 0.1);

            // 出线槽
            translate([-3, -puck_od / 2 + 1, -0.1]) cube([6, puck_od / 2 - 16, 4.1]);

            // 挂绳孔（穿过耳片）
            translate([0, -puck_od / 2 - 0.5, -1]) cylinder(d = 3.4, h = puck_h + 2);
        }
}

module foam() {
    translate([0, 0, puck_face_z]) difference() {
        cylinder(d = foam_d, h = foam_t);
        translate([0, 0, -0.1]) cylinder(d = 40, h = foam_t + 0.2);
    }
}

// ================= 渲染 =================
if (PART == "plate") {
    plate();
} else if (PART == "puck") {
    translate([0, 0, -puck_z0]) puck();
} else if (PART == "foam") {
    translate([0, 0, -puck_face_z]) foam();
} else {
    // 装配预览
    color([0.75, 0.78, 0.82]) plate();
    color([0.95, 0.95, 0.95, 0.7]) foam();
    color([0.55, 0.75, 0.95]) puck();
    // 弹簧示意
    for (a = float_angles) rotate([0, 0, a]) translate([float_screw_r, 0, plate_t])
        cylinder(d = 4.8, h = float_gap, center = false);
}
