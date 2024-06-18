#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;
    int o_x, o_y;
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];
    /* coordinates transformed in simulate.cpp */

    int quadrotor_size[2] = { 160,15 };
    int connector_size[2] = { 4,25 };
    int propeller_size[2] = { 15,5 };

	int quadrotor_color = 0xFF3399FF;    //kolory s¹ w kolejnoœci ABGR
    int connector_color = 0xFFa37575;
    static unsigned int propeller_color[2] = { 0xFFff661a,0xFFFF9966 };

    float quadrotor_left[2] = { q_x - quadrotor_size[0] / 2 * cos(q_theta),q_y - quadrotor_size[0] / 2 * sin(q_theta) };
    float quadrotor_right[2] = { q_x + quadrotor_size[0] / 2 * cos(q_theta),q_y + quadrotor_size[0] / 2 * sin(q_theta) };

    float connector_distance[2] = { quadrotor_size[0] * 0.9f * cos(q_theta),quadrotor_size[0] * 0.9f * sin(q_theta) };
    float connector_length[2] = { -connector_size[1] * sin(q_theta), connector_size[1] * cos(q_theta) };

    float connector_left_1[2] = { quadrotor_right[0] - connector_distance[0],quadrotor_right[1] - connector_distance[1] };
    float connector_left_2[2] = { quadrotor_right[0] - connector_distance[0] - connector_length[0],quadrotor_right[1] - connector_distance[1] - connector_length[1] };
    float connector_right_1[2] = { quadrotor_left[0] + connector_distance[0],quadrotor_left[1] + connector_distance[1] };
    float connector_right_2[2] = { quadrotor_left[0] + connector_distance[0] - connector_length[0],quadrotor_left[1] + connector_distance[1] - connector_length[1] };

    static int animation_duration = 0;
    static bool propeller_in_motion = false;
    int t = SDL_GetTicks();
    if (t - animation_duration > 100) {

        animation_duration = SDL_GetTicks();
        unsigned int z = propeller_color[0];
        propeller_color[0] = propeller_color[1];
        propeller_color[1] = z;

    }

    float propeller_left_1[2] = { connector_left_2[0] + (propeller_size[0]), connector_left_2[1] };
    float propeller_left_2[2] = { connector_left_2[0] - (propeller_size[0]), connector_left_2[1] };
    float propeller_right_1[2] = { connector_right_2[0] + (propeller_size[0]), connector_right_2[1] };
    float propeller_right_2[2] = { connector_right_2[0] - (propeller_size[0]), connector_right_2[1] };

    thickLineColor(gRenderer.get(), quadrotor_left[0], quadrotor_left[1], quadrotor_right[0], quadrotor_right[1], quadrotor_size[1], quadrotor_color);

    thickLineColor(gRenderer.get(), connector_left_1[0], connector_left_1[1], connector_left_2[0], connector_left_2[1], connector_size[0], connector_color);
    thickLineColor(gRenderer.get(), connector_right_1[0], connector_right_1[1], connector_right_2[0], connector_right_2[1], connector_size[0], connector_color);

    filledEllipseColor(gRenderer.get(), propeller_left_1[0], propeller_left_1[1], propeller_size[0], propeller_size[1], propeller_color[0]);
    filledEllipseColor(gRenderer.get(), propeller_right_1[0], propeller_right_1[1], propeller_size[0], propeller_size[1], propeller_color[0]);
    filledEllipseColor(gRenderer.get(), propeller_left_2[0], propeller_left_2[1], propeller_size[0], propeller_size[1], propeller_color[1]);
    filledEllipseColor(gRenderer.get(), propeller_right_2[0], propeller_right_2[1], propeller_size[0], propeller_size[1], propeller_color[1]);
}