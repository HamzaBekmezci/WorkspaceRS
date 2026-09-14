#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "kinematics.h"
#include "sim_set.h"

// Otopilot sistemini sıfırlar (yeni bir simülasyon başladığında çağrılır)
void autopilot_init(void);

// Otopilota yeni bir hedef konum (waypoint) atar
void autopilot_set_waypoint(float x, float y, float z);

// Her simülasyon adımında (dt) çağrılarak gerekli kuvvet ve torkları hesaplar
void autopilot_step(KinematicState_t *state, RigidBodyParams_t *body, float dt);

#endif // AUTOPILOT_H