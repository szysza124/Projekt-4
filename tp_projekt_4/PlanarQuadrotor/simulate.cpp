/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <thread>
#include <matplot/matplot.h>
#include <SDL/include/SDL.h>

//dŸwiêk silników
void generateEngineSound(Uint8* buffer, int length, double frequency, double left_gain) {
    const int amplitude = 10; // Amplituda dŸwiêku
    const double sampleRate = 100.0; // Czêstotliwoœæ próbkowania

    for (int i = 0; i < length; ++i) {
        double time = i / sampleRate;
        double value = amplitude * sin(2.0 * M_PI * frequency * time);

        // Zmiana amplitudy w zale¿noœci od kierunku ruchu
        if (left_gain > 1.0)
            buffer[i] = (Uint8)(value * left_gain + 2); // Lewy g³oœnik g³oœniejszy
        else if (left_gain < 1.0)
            buffer[i] = (Uint8)(value + 2 * left_gain); // Prawy g³oœnik g³oœniejszy
        else
            buffer[i] = (Uint8)(value + 2); // Równa amplituda dla obu g³oœników
    }
}

// Definicja funkcji rysuj¹cej trajektoriê drona w trzech ró¿nych p³aszczyznach: X, Y i Theta w czasie
void rysuj(std::vector<float> x_history, std::vector<float> y_history, std::vector<float> theta_history, std::vector<float> time) {
    // Tworzenie trzech podwykresów u³o¿onych jeden pod drugim (3 wiersze, 1 kolumna) w oknie wykresu
    matplot::subplot(3, 1, 0); // Pierwszy podwykres - pozycja X
    // Tworzenie wykresu pozycji X w zale¿noœci od czasu
    matplot::plot(time, x_history);
    matplot::title("X"); // Dodanie tytu³u "X" do wykresu

    matplot::subplot(3, 1, 1); // Drugi podwykres - pozycja Y
    // Tworzenie wykresu pozycji Y w zale¿noœci od czasu
    matplot::plot(time, y_history);
    matplot::title("Y"); // Dodanie tytu³u "Y" do wykresu

    matplot::subplot(3, 1, 2); // Trzeci podwykres - k¹t Theta
    // Tworzenie wykresu k¹ta Theta w zale¿noœci od czasu
    matplot::plot(time, theta_history);
    matplot::title("Theta"); // Dodanie tytu³u "Theta" do wykresu

    matplot::show(); // Wyœwietlenie wygenerowanych wykresów w oknie wykresu
}


Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    //LQR
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 0.004, 0.004, 400, 0.005, 0.045, 2 / 2 / M_PI;
    R.row(0) << 30, 7;
    R.row(1) << 7, 30;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int main(int argc, char* args[])
{
    // Inicjalizacja SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    // Konfiguracja specyfikacji audio
    SDL_AudioSpec spec;
    spec.freq = 54100; // Czêstotliwoœæ próbkowania
    spec.format = AUDIO_U8; // Format dŸwiêku
    spec.channels = 2; // Liczba kana³ów (stereo)
    spec.samples = 8096; // Rozmiar bufora próbek
    spec.callback = NULL; // Brak funkcji callback
    spec.userdata = NULL;

    // Otwarcie urz¹dzenia audio
    SDL_AudioDeviceID deviceId = SDL_OpenAudioDevice(NULL, 0, &spec, NULL, 0);
    if (deviceId == 0) {
        std::cerr << "Failed to open audio device: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return -1;
    }

    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;
    const int start_x = 640;
    const int start_y = 360;

    /**
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * 2. Update PlanarQuadrotor from simulation when goal is changed
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    initial_state << start_x, start_y, 0, 0, 0, 0;
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.01;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    // Tworzenie wektorów historii pozycji X, Y, k¹ta Theta oraz czasu,
    // zainicjowanych pocz¹tkowymi wartoœciami
    std::vector<float> x_history{ float(start_x) }; // Wektor historii pozycji X, inicjowany wartoœci¹ start_x
    std::vector<float> y_history{ float(start_y) }; // Wektor historii pozycji Y, inicjowany wartoœci¹ start_y
    std::vector<float> theta_history{ 0 }; // Wektor historii k¹ta Theta, inicjowany wartoœci¹ 0
    std::vector<float> time{ 0 }; // Wektor czasu, inicjowany wartoœci¹ 0

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        float probki = 0;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            //events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else
                {
                    if (e.type == SDL_MOUSEMOTION)
                    {
                        SDL_GetMouseState(&x, &y);
                        std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                    }
                    if (e.type == SDL_MOUSEBUTTONDOWN) {
                        SDL_GetMouseState(&x, &y);
                        goal_state << x, y, 0, 0, 0, 0;
                        std::cout << "Mouse left-click " << x << " " << y << std::endl;
                        quadrotor.SetGoal(goal_state);
                    }
                    if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p) {
                        std::thread p(rysuj, x_history, y_history, theta_history, time);
                        p.detach();
                    }
                }

            }

            SDL_Delay((int)dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Symulacja drona w przód w czasie */
            control(quadrotor, K); // Wywo³anie funkcji kontroluj¹cej drona
            quadrotor.Update(dt); // Aktualizacja stanu drona

            // Pêtla while sprawdzaj¹ca, czy dron osi¹gn¹³ now¹ pozycjê w odleg³oœci wiêkszej ni¿ 0.2 od poprzedniej
            while (abs(x_history.back() - quadrotor.GetState()[0]) > 0.2 || abs(y_history.back() - quadrotor.GetState()[1]) > 0.2) {
                // Warunek sprawdzaj¹cy, czy nie przekroczono limitu danych w historii
                if (probki > 8000) {
                    // Usuniêcie najstarszych danych z wektorów historii
                    time.erase(time.begin()); // Usuniêcie czasu
                    x_history.erase(x_history.begin()); // Usuniêcie pozycji X
                    y_history.erase(y_history.begin()); // Usuniêcie pozycji Y
                    theta_history.erase(theta_history.begin()); // Usuniêcie k¹ta Theta
                }
                // Dodanie nowych danych do wektorów historii
                time.push_back(++probki); // Dodanie nowego czasu
                x_history.push_back(quadrotor.GetState()[0]); // Dodanie nowej pozycji X drona
                y_history.push_back(quadrotor.GetState()[1]); // Dodanie nowej pozycji Y drona
                theta_history.push_back(quadrotor.GetState()[2]); // Dodanie nowego k¹ta Theta drona
            }

            // Generowanie dŸwiêku silnika na podstawie ruchu quadrotora
            double state_theta = quadrotor.GetState()[2]; // Pobierz k¹t theta
            double left_gain = 1 + std::abs(state_theta) / M_PI; // Oblicz wspó³czynnik wzmocnienia dla lewego g³oœnika
            double frequency = 150; // Bazowa czêstotliwoœæ dŸwiêku
            Uint8* audioBuffer = new Uint8[spec.samples];
            generateEngineSound(audioBuffer, spec.samples, frequency, left_gain);
            SDL_QueueAudio(deviceId, audioBuffer, spec.samples);
            SDL_PauseAudioDevice(deviceId, 0);
            delete[] audioBuffer;
        }

    }
    // Zwalnianie zasobów SDL
    SDL_CloseAudioDevice(deviceId);
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}