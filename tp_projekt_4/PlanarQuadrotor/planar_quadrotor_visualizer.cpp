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
    q_theta = -1*state[2];//poprawka k¹ta aby dron wygl¹da³ naturalnie 
    /* coordinates transformed in simulate.cpp */

    int korpus_rozmiar[2] = { 160,15 };
    int patyk_rozmiar[2] = { 4,25 };
    int smiglo_rozmiar[2] = { 15,5 };

	int korpus_kolor = 0xFF3399FF;    //kolory s¹ w kolejnoœci ABGR
    int patyk_kolor = 0xFFa37575;
    static unsigned int smiglo_kolory[2] = { 0xFFff661a,0xFFFF9966 };

    float korpus_lewy[2] = { q_x - korpus_rozmiar[0] / 2 * cos(q_theta),q_y - korpus_rozmiar[0] / 2 * sin(q_theta) };
    float korpus_prawy[2] = { q_x + korpus_rozmiar[0] / 2 * cos(q_theta),q_y + korpus_rozmiar[0] / 2 * sin(q_theta) };

    float patyk_odleglosc[2] = { korpus_rozmiar[0] * 0.9f * cos(q_theta),korpus_rozmiar[0] * 0.9f * sin(q_theta) };
    float patyk_dlugosc[2] = { -patyk_rozmiar[1] * sin(q_theta), patyk_rozmiar[1] * cos(q_theta) };

    float patyk_lewy_1[2] = { korpus_prawy[0] - patyk_odleglosc[0],korpus_prawy[1] - patyk_odleglosc[1] };
    float patyk_lewy_2[2] = { korpus_prawy[0] - patyk_odleglosc[0] - patyk_dlugosc[0],korpus_prawy[1] - patyk_odleglosc[1] - patyk_dlugosc[1] };
    float patyk_prawy_1[2] = { korpus_lewy[0] + patyk_odleglosc[0],korpus_lewy[1] + patyk_odleglosc[1] };
    float patyk_prawy_2[2] = { korpus_lewy[0] + patyk_odleglosc[0] - patyk_dlugosc[0],korpus_lewy[1] + patyk_odleglosc[1] - patyk_dlugosc[1] };

    static int czas_animacji = 0;
    int t = SDL_GetTicks();
    if (t - czas_animacji > 100) {

        czas_animacji = SDL_GetTicks();
        unsigned int z = smiglo_kolory[0];
        smiglo_kolory[0] = smiglo_kolory[1];
        smiglo_kolory[1] = z;

    }

    float smiglo_lewe_1[2] = { patyk_lewy_2[0] + (smiglo_rozmiar[0]), patyk_lewy_2[1] };
    float smiglo_lewe_2[2] = { patyk_lewy_2[0] - (smiglo_rozmiar[0]), patyk_lewy_2[1] };
    float smiglo_prawe_1[2] = { patyk_prawy_2[0] + (smiglo_rozmiar[0]), patyk_prawy_2[1] };
    float smiglo_prawe_2[2] = { patyk_prawy_2[0] - (smiglo_rozmiar[0]), patyk_prawy_2[1] };

	thickLineColor(gRenderer.get(), korpus_lewy[0], korpus_lewy[1], korpus_prawy[0], korpus_prawy[1], korpus_rozmiar[1], korpus_kolor);    //rysowanie korpusu

	thickLineColor(gRenderer.get(), patyk_lewy_1[0], patyk_lewy_1[1], patyk_lewy_2[0], patyk_lewy_2[1], patyk_rozmiar[0], patyk_kolor);         //rysowanie ³¹czników
    thickLineColor(gRenderer.get(), patyk_prawy_1[0], patyk_prawy_1[1], patyk_prawy_2[0], patyk_prawy_2[1], patyk_rozmiar[0], patyk_kolor);

	filledEllipseColor(gRenderer.get(), smiglo_lewe_1[0], smiglo_lewe_1[1], smiglo_rozmiar[0], smiglo_rozmiar[1], smiglo_kolory[0]);     //rysowanie œmigie³
    filledEllipseColor(gRenderer.get(), smiglo_prawe_1[0], smiglo_prawe_1[1], smiglo_rozmiar[0], smiglo_rozmiar[1], smiglo_kolory[0]);
    filledEllipseColor(gRenderer.get(), smiglo_lewe_2[0], smiglo_lewe_2[1], smiglo_rozmiar[0], smiglo_rozmiar[1], smiglo_kolory[1]);
    filledEllipseColor(gRenderer.get(), smiglo_prawe_2[0], smiglo_prawe_2[1], smiglo_rozmiar[0], smiglo_rozmiar[1], smiglo_kolory[1]);
}