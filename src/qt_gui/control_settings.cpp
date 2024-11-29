// SPDX-FileCopyrightText: Copyright 2024 shadPS4 Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include <fstream>
#include <map>
#include <toml.hpp>

#include "common/path_util.h"

#include "control_settings.h"
#include "ui_control_settings.h"
#include "../sdl_window.h"

QStringList Inputs =    {"cross",
                        "circle",
                        "square",
                        "triangle",
                        "L1",
                        "R1",
                        "L2",
                        "R2",
                        "L3",
                        "R3",
                        "options",
                        "dpad_up",
                        "dpad_down",
                        "dpad_left",
                        "dpad_right",
                        "lstickup",
                        "lstickdown",
                        "lstickleft",
                        "lstickright",
                        "rstickup",
                        "rstickdown",
                        "rstickleft",
                        "rstickright"};


QStringList InputsButtons   {"cross",
                             "circle",
                             "square",
                             "triangle",
                             "L1",
                             "R1",
                             "L2",
                             "R2",
                             "L3",
                             "R3",
                             "options",
                             "dpad up",
                             "dpad down",
                             "dpad left",
                             "dpad right"};


static std::string Amap = "cross";
static std::string Ymap = "triangle";
static std::string Xmap = "square"; 
static std::string Bmap = "circle";
static std::string LBmap = "L1";
static std::string RBmap = "R1";
static std::string dupmap = "dpad_up";
static std::string ddownmap = "dpad_down";
static std::string dleftmap = "dpad_left";
static std::string drightmap = "dpad_right";
static std::string rstickmap = "R3";    
static std::string lstickmap = "L3";
static std::string startmap = "options";
static std::string LTmap = "L2";
static std::string RTmap = "R2";
static std::string Lstickupmap = "lstickup";
static std::string Lstickdownmap = "lstickdown";
static std::string Lstickleftmap = "lstickleft"; 
static std::string Lstickrightmap = "lstickright"; 
static std::string Rstickupmap = "rstickup"; 
static std::string Rstickdownmap = "rstickdown"; 
static std::string Rstickleftmap = "rstickleft"; 
static std::string Rstickrightmap = "rstickright"; 

static bool Lstickbuttonsmap = false;
static bool Lstickswapmap = false;
static bool LstickinvertYmap = false;
static bool LstickinvertXmap = false;
static bool Rstickbuttonsmap = false;
static bool Rstickswapmap = false;
static bool RstickinvertYmap = false;
static bool RstickinvertXmap =  false;


ControlSettings::ControlSettings(QWidget* parent) : QDialog(parent) , ui(new Ui::ControlSettings) {

    ui->setupUi(this);
    ui->DpadUpBox->addItems(Inputs);
    ui->DpadDownBox->addItems(Inputs);
    ui->DpadLeftBox->addItems(Inputs);
    ui->DpadRightBox->addItems(Inputs);
    ui->LBBox->addItems(Inputs);
    ui->RBBox->addItems(Inputs);
    ui->LTBox->addItems(Inputs);
    ui->RTBox->addItems(Inputs);
    ui->RClickBox->addItems(Inputs);
    ui->LClickBox->addItems(Inputs);
    ui->StartBox->addItems(Inputs);
    ui->ABox->addItems(Inputs);
    ui->BBox->addItems(Inputs);
    ui->XBox->addItems(Inputs);
    ui->YBox->addItems(Inputs);

    ui->LStickUpBox->addItems(InputsButtons);
    ui->LStickDownBox->addItems(InputsButtons);
    ui->LStickLeftBox->addItems(InputsButtons);
    ui->LStickRightBox->addItems(InputsButtons);
    ui->RStickUpBox->addItems(InputsButtons);
    ui->RStickDownBox->addItems(InputsButtons);
    ui->RStickLeftBox->addItems(InputsButtons);
    ui->RStickRightBox->addItems(InputsButtons);

    toml::value data = toml::parse("Controller.toml");
    Amap = toml::find_or<std::string>(data, "A_button", "remap","cross");
    Ymap = toml::find_or<std::string>(data, "Y_button", "remap", "triangle");
    Xmap = toml::find_or<std::string>(data, "X_button", "remap", "square");
    Bmap = toml::find_or<std::string>(data, "B_button", "remap", "circle");
    LBmap = toml::find_or<std::string>(data, "Left_bumper", "remap", "L1");
    RBmap = toml::find_or<std::string>(data, "Right_bumper", "remap", "R1");
    dupmap = toml::find_or<std::string>(data, "dpad_up", "remap","dpad_up");
    ddownmap = toml::find_or<std::string>(data, "dpad_down", "remap", "dpad_down");
    dleftmap = toml::find_or<std::string>(data, "dpad_left", "remap", "dpad_left");
    drightmap = toml::find_or<std::string>(data, "dpad_right", "remap", "dpad_right");
    rstickmap = toml::find_or<std::string>(data, "Right_stick_button", "remap", "R3");
    lstickmap = toml::find_or<std::string>(data, "Left_stick_button", "remap", "L3");
    startmap = toml::find_or<std::string>(data, "Start", "remap", "options");
    LTmap = toml::find_or<std::string>(data, "Left_trigger", "remap", "L2");
    RTmap = toml::find_or<std::string>(data, "Right_trigger", "remap", "R2");
    Lstickupmap = toml::find_or<std::string>(data, "If_Left_analog_stick_mapped_to_buttons", "Left_stick_up_remap", "lstickup");
    Lstickdownmap = toml::find_or<std::string>(data, "If_Left_analog_stick_mapped_to_buttons", "Left_stick_down_remap", "lstickdown");
    Lstickleftmap = toml::find_or<std::string>(data, "If_Left_analog_stick_mapped_to_buttons", "Left_stick_left_remap", "lstickleft");
    Lstickrightmap = toml::find_or<std::string>(data, "If_Left_analog_stick_mapped_to_buttons", "Left_stick_right_remap", "lstickright"); 
    Rstickupmap = toml::find_or<std::string>(data, "If_Right_analog_stick_mapped_to_buttons", "Right_stick_up_remap", "rstickup");
    Rstickdownmap = toml::find_or<std::string>(data, "If_Right_analog_stick_mapped_to_buttons", "Right_stick_down_remap", "rstickdown");
    Rstickleftmap = toml::find_or<std::string>(data, "If_Right_analog_stick_mapped_to_buttons", "Right_stick_left_remap", "rstickleft");
    Rstickrightmap = toml::find_or<std::string>(data, "If_Right_analog_stick_mapped_to_buttons", "Right_stick_right_remap", "rstickright");

    Lstickbuttonsmap = toml::find_or<bool>(data, "Left_analog_stick_behavior", "Mapped_to_buttons", false);
    Lstickswapmap = toml::find_or<bool>(data, "Left_analog_stick_behavior", "Swap_sticks", false);
    LstickinvertYmap = toml::find_or<bool>(data, "Left_analog_stick_behavior", "Invert_movement_vertical", false);
    LstickinvertXmap = toml::find_or<bool>(data, "Left_analog_stick_behavior", "Invert_movement_horizontal", false);
    Rstickbuttonsmap = toml::find_or<bool>(data, "Right_analog_stick_behavior", "Mapped_to_buttons", false);
    Rstickswapmap = toml::find_or<bool>(data, "Right_analog_stick_behavior", "Swap_sticks", false);
    RstickinvertYmap = toml::find_or<bool>(data, "Right_analog_stick_behavior", "Invert_movement_vertical", false);
    RstickinvertXmap = toml::find_or<bool>(data, "Right_analog_stick_behavior", "Invert_movement_horizontal", false);

std::map<std::string, int> index_map = {
        {"cross", 0},
        {"circle", 1},
        {"square", 2},
        {"triangle", 3},
        {"L1", 4},
        {"R1", 5},
        {"L2", 6},
        {"R2", 7},
        {"L3", 8},
        {"R3", 9},
        {"options", 10},
        {"dpad_up", 11},
        {"dpad_down", 12},
        {"dpad_left", 13},
        {"dpad_right", 14},
        {"lstickup", 15},
        {"lstickdown", 16}, 
        {"lstickleft", 17},
        {"lstickright", 18}, 
        {"rstickup", 19}, 
        {"rstickdown", 20}, 
        {"rstickleft", 21},
        {"rstickright", 22}, 
        };

ui->ABox->setCurrentIndex(index_map[Amap]);
ui->BBox->setCurrentIndex(index_map[Bmap]);
ui->XBox->setCurrentIndex(index_map[Xmap]);
ui->YBox->setCurrentIndex(index_map[Ymap]);
ui->DpadUpBox->setCurrentIndex(index_map[dupmap]);
ui->DpadDownBox->setCurrentIndex(index_map[ddownmap]);
ui->DpadLeftBox->setCurrentIndex(index_map[dleftmap]);
ui->DpadRightBox->setCurrentIndex(index_map[drightmap]);
ui->LClickBox->setCurrentIndex(index_map[lstickmap]);
ui->RClickBox->setCurrentIndex(index_map[rstickmap]);
ui->LBBox->setCurrentIndex(index_map[LBmap]);
ui->RBBox->setCurrentIndex(index_map[RBmap]);
ui->LTBox->setCurrentIndex(index_map[LTmap]);
ui->RTBox->setCurrentIndex(index_map[RTmap]);
ui->LStickUpBox->setCurrentIndex(index_map[Lstickupmap]);
ui->LStickDownBox->setCurrentIndex(index_map[Lstickdownmap]);
ui->LStickLeftBox->setCurrentIndex(index_map[Lstickleftmap]);
ui->LStickRightBox->setCurrentIndex(index_map[Lstickrightmap]);
ui->RStickUpBox->setCurrentIndex(index_map[Rstickupmap]);
ui->RStickDownBox->setCurrentIndex(index_map[Rstickdownmap]);
ui->RStickLeftBox->setCurrentIndex(index_map[Rstickleftmap]);
ui->RStickRightBox->setCurrentIndex(index_map[Rstickrightmap]);
ui->StartBox->setCurrentIndex(index_map[startmap]);

ui->LStickButtons->setChecked(Lstickbuttonsmap);
ui->LStickInvertX->setChecked(LstickinvertXmap);
ui->LStickInvertY->setChecked(LstickinvertYmap);
ui->LStickSwap->setChecked(Lstickswapmap);
ui->RStickButtons->setChecked(Rstickbuttonsmap);
ui->RStickInvertX->setChecked(RstickinvertXmap);
ui->RStickInvertY->setChecked(RstickinvertYmap);
ui->RStickSwap->setChecked(Rstickswapmap);

connect(ui->LStickButtons, &QCheckBox::stateChanged, this,
              [](bool val) { Lstickbuttonsmap = val; });
connect(ui->LStickInvertX, &QCheckBox::stateChanged, this,
              [](bool val) { LstickinvertXmap = val; });
connect(ui->LStickInvertY, &QCheckBox::stateChanged, this,
              [](bool val) { LstickinvertYmap = val; });
connect(ui->LStickSwap, &QCheckBox::stateChanged, this,
              [](bool val) { Lstickswapmap = val; });
connect(ui->RStickButtons, &QCheckBox::stateChanged, this,
              [](bool val) { Rstickbuttonsmap = val; });
connect(ui->RStickInvertX, &QCheckBox::stateChanged, this,
              [](bool val) { RstickinvertXmap = val; });
connect(ui->RStickInvertY, &QCheckBox::stateChanged, this,
              [](bool val) { RstickinvertYmap = val; });
connect(ui->RStickSwap, &QCheckBox::stateChanged, this,
              [](bool val) { Rstickswapmap = val; });

connect(ui->DpadUpBox, &QComboBox::currentTextChanged, this,
               [](const QString& DUptext) { dupmap = DUptext.toStdString(); });
connect(ui->DpadDownBox, &QComboBox::currentTextChanged, this,
               [](const QString& DDowntext) { ddownmap = DDowntext.toStdString(); });
connect(ui->DpadLeftBox, &QComboBox::currentTextChanged, this,
               [](const QString& DLefttext) { dleftmap = DLefttext.toStdString(); });
connect(ui->DpadRightBox, &QComboBox::currentTextChanged, this,
               [](const QString& DRighttext) { drightmap = DRighttext.toStdString(); });
connect(ui->ABox, &QComboBox::currentTextChanged, this,
               [](const QString& Atext) { Amap = Atext.toStdString(); });
connect(ui->BBox, &QComboBox::currentTextChanged, this,
               [](const QString& Btext) { Bmap = Btext.toStdString(); });
connect(ui->XBox, &QComboBox::currentTextChanged, this,
               [](const QString& Xtext) { Xmap = Xtext.toStdString(); });
connect(ui->YBox, &QComboBox::currentTextChanged, this,
               [](const QString& Ytext) { Ymap = Ytext.toStdString(); });
connect(ui->LBBox, &QComboBox::currentTextChanged, this,
               [](const QString& LBtext) { LBmap = LBtext.toStdString(); });
connect(ui->RBBox, &QComboBox::currentTextChanged, this,
               [](const QString& RBtext) { RBmap = RBtext.toStdString(); });
connect(ui->LTBox, &QComboBox::currentTextChanged, this,
               [](const QString& LTtext) { LTmap = LTtext.toStdString(); });
connect(ui->RTBox, &QComboBox::currentTextChanged, this,
               [](const QString& RTtext) { RTmap = RTtext.toStdString(); });
connect(ui->LClickBox, &QComboBox::currentTextChanged, this,
               [](const QString& lsticktext) { lstickmap = lsticktext.toStdString(); });
connect(ui->RClickBox, &QComboBox::currentTextChanged, this,
               [](const QString& rsticktext) { rstickmap = rsticktext.toStdString(); });
connect(ui->StartBox, &QComboBox::currentTextChanged, this,
               [](const QString& starttext) { startmap = starttext.toStdString(); });

connect(ui->LStickUpBox, &QComboBox::currentTextChanged, this,
               [](const QString& Lstickuptext) { Lstickupmap = Lstickuptext.toStdString(); });
connect(ui->LStickDownBox, &QComboBox::currentTextChanged, this,
               [](const QString& Lstickdowntext) { Lstickdownmap = Lstickdowntext.toStdString(); });
connect(ui->LStickLeftBox, &QComboBox::currentTextChanged, this,
               [](const QString& Lsticklefttext) { Lstickleftmap = Lsticklefttext.toStdString(); });
connect(ui->LStickRightBox, &QComboBox::currentTextChanged, this,
               [](const QString& Lstickrighttext) { Lstickrightmap = Lstickrighttext.toStdString(); });
connect(ui->RStickUpBox, &QComboBox::currentTextChanged, this,
               [](const QString& Rstickuptext) { Rstickupmap = Rstickuptext.toStdString(); });
connect(ui->RStickDownBox, &QComboBox::currentTextChanged, this,
               [](const QString& Rstickdowntext) { Rstickdownmap = Rstickdowntext.toStdString(); });
connect(ui->RStickLeftBox, &QComboBox::currentTextChanged, this,
               [](const QString& Rsticklefttext) { Rstickleftmap = Rsticklefttext.toStdString(); });
connect(ui->RStickRightBox, &QComboBox::currentTextChanged, this,
               [](const QString& Rstickrighttext) { Rstickrightmap = Rstickrighttext.toStdString(); });

connect(ui->buttonBox, &QDialogButtonBox::clicked, this,
            [this](QAbstractButton* button) {
                if (button == ui->buttonBox->button(QDialogButtonBox::Save)) {
                    SaveControllerConfig();
                    QWidget::close();
                } else if (button == ui->buttonBox->button(QDialogButtonBox::RestoreDefaults)) {
                    SetDefault();
                }
            });

connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &QWidget::close);
}

void ControlSettings::SaveControllerConfig() {

    toml::value data = toml::parse("Controller.toml");
    
    data["A_button"]["remap"] = Amap;
    data["Y_button"]["remap"] = Ymap;
    data["X_button"]["remap"] = Xmap;
    data["B_button"]["remap"] = Bmap;
    data["Left_bumper"]["remap"] = LBmap;
    data["Right_bumper"]["remap"] = RBmap;
    data["Left_trigger"]["remap"] = LTmap;
    data["Right_trigger"]["remap"] = RTmap;
    data["dpad_up"]["remap"] = dupmap;
    data["dpad_down"]["remap"] = ddownmap;
    data["dpad_left"]["remap"] = dleftmap;
    data["dpad_right"]["remap"] = drightmap;
    data["Left_stick_button"]["remap"] = lstickmap;
    data["Right_stick_button"]["remap"] = rstickmap;
    data["Start"]["remap"] = startmap;
    data["If_Left_analog_stick_mapped_to_buttons"]["Left_stick_up_remap"] = Lstickupmap;
    data["If_Left_analog_stick_mapped_to_buttons"]["Left_stick_down_remap"] = Lstickdownmap;
    data["If_Left_analog_stick_mapped_to_buttons"]["Left_stick_left_remap"] = Lstickleftmap;
    data["If_Left_analog_stick_mapped_to_buttons"]["Left_stick_right_remap"] = Lstickrightmap;
    data["If_Right_analog_stick_mapped_to_buttons"]["Right_stick_up_remap"] = Rstickupmap;
    data["If_Right_analog_stick_mapped_to_buttons"]["Right_stick_down_remap"] = Rstickdownmap;
    data["If_Right_analog_stick_mapped_to_buttons"]["Right_stick_left_remap"] = Rstickleftmap;
    data["If_Right_analog_stick_mapped_to_buttons"]["Right_stick_right_remap"] = Rstickrightmap; 

    data["Left_analog_stick_behavior"]["Mapped_to_buttons"] = Lstickbuttonsmap;
    data["Left_analog_stick_behavior"]["Swap_sticks"] = Lstickswapmap;
    data["Left_analog_stick_behavior"]["Invert_movement_vertical"] = LstickinvertYmap;
    data["Left_analog_stick_behavior"]["Invert_movement_horizontal"] = LstickinvertXmap;   
 
    data["Right_analog_stick_behavior"]["Mapped_to_buttons"] = Rstickbuttonsmap;
    data["Right_analog_stick_behavior"]["Swap_sticks"] = Rstickswapmap;
    data["Right_analog_stick_behavior"]["Invert_movement_vertical"] = RstickinvertYmap;
    data["Right_analog_stick_behavior"]["Invert_movement_horizontal"] = RstickinvertXmap;


    std::ofstream remaptoml("Controller.toml");
    remaptoml << data;
    remaptoml.close();

    Frontend::RefreshMappings();
}

void ControlSettings::SetDefault() {

ui->ABox->setCurrentIndex(0);
ui->BBox->setCurrentIndex(1);
ui->XBox->setCurrentIndex(2);
ui->YBox->setCurrentIndex(3);
ui->DpadUpBox->setCurrentIndex(11);
ui->DpadDownBox->setCurrentIndex(12);
ui->DpadLeftBox->setCurrentIndex(13);
ui->DpadRightBox->setCurrentIndex(14);
ui->LClickBox->setCurrentIndex(8);
ui->RClickBox->setCurrentIndex(9);
ui->LBBox->setCurrentIndex(4);
ui->RBBox->setCurrentIndex(5);
ui->LTBox->setCurrentIndex(6);
ui->RTBox->setCurrentIndex(7);
ui->StartBox->setCurrentIndex(10);

ui->LStickUpBox->setCurrentIndex(11);
ui->LStickDownBox->setCurrentIndex(12);
ui->LStickLeftBox->setCurrentIndex(13);
ui->LStickRightBox->setCurrentIndex(14);
ui->RStickUpBox->setCurrentIndex(3);
ui->RStickDownBox->setCurrentIndex(0);
ui->RStickLeftBox->setCurrentIndex(2);
ui->RStickRightBox->setCurrentIndex(1);

ui->LStickButtons->setChecked(false);
ui->LStickInvertX->setChecked(false);
ui->LStickInvertY->setChecked(false);
ui->LStickSwap->setChecked(false);
ui->RStickButtons->setChecked(false);
ui->RStickInvertX->setChecked(false);
ui->RStickInvertY->setChecked(false);
ui->RStickSwap->setChecked(false);
}

ControlSettings::~ControlSettings() { } // empty desctructor