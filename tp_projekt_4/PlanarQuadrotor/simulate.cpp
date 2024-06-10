/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <thread>
#include <matplot/matplot.h>
#include <SDL/include/SDL.h>

//d�wi�k silnik�w
void generateEngineSound(Uint8* buffer, int length, double frequency, double left_gain) {
    const int amplitude = 10; // Amplituda d�wi�ku
    const double sampleRate = 100.0; // Cz�stotliwo�� pr�bkowania

    for (int i = 0; i < length; ++i) {
        double time = i / sampleRate;
        double value = amplitude * sin(2.0 * M_PI * frequency * time);

        // Zmiana amplitudy w zale�no�ci od kierunku ruchu
        if (left_gain > 1.0)
            buffer[i] = (Uint8)(value * left_gain + 2); // Lewy g�o�nik g�o�niejszy
        else if (left_gain < 1.0)
            buffer[i] = (Uint8)(value + 2 * left_gain); // Prawy g�o�nik g�o�niejszy
        else
            buffer[i] = (Uint8)(value + 2); // R�wna amplituda dla obu g�o�nik�w
    }
}

// Definicja funkcji rysuj�cej trajektori� drona w trzech r�nych p�aszczyznach: X, Y i Theta w czasie
void rysuj(std::vector<float> x_history, std::vector<float> y_history, std::vector<float> theta_history, std::vector<float> time) {
    // Tworzenie trzech podwykres�w u�o�onych jeden pod drugim (3 wiersze, 1 kolumna) w oknie wykresu
    matplot::subplot(3, 1, 0); // Pierwszy podwykres - pozycja X
    // Tworzenie wykresu pozycji X w zale�no�ci od czasu
    matplot::plot(time, x_history);
    matplot::title("X"); // Dodanie tytu�u "X" do wykresu

    matplot::subplot(3, 1, 1); // Drugi podwykres - pozycja Y
    // Tworzenie wykresu pozycji Y w zale�no�ci od czasu
    matplot::plot(time, y_history);
    matplot::title("Y"); // Dodanie tytu�u "Y" do wykresu

    matplot::subplot(3, 1, 2); // Trzeci podwykres - k�t Theta
    // Tworzenie wykresu k�ta Theta w zale�no�ci od czasu
    matplot::plot(time, theta_history);
    matplot::title("Theta"); // Dodanie tytu�u "Theta" do wykresu

    matplot::show(); // Wy�wietlenie wygenerowanych wykres�w w oknie wykresu
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

    Q.diagonal() << 0.004, 0.004, 400, 0.005, 0.045, 2. / 2 / M_PI;
    R.row(0) << 3, 0.7;
    R.row(1) << 0.7, 3;

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
    spec.freq = 54100; // Cz�stotliwo�� pr�bkowania
    spec.format = AUDIO_U8; // Format d�wi�ku
    spec.channels = 2; // Liczba kana��w (stereo)
    spec.samples = 8096; // Rozmiar bufora pr�bek
    spec.callback = NULL; // Brak funkcji callback
    spec.userdata = NULL;

    // Otwarcie urz�dzenia audio
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
    const float scale_factor = 0.01f; // Adjust based on your simulation's scale
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
    //goal_state << 0, 0, 0, 0, 0, 0;
    //quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.1 ;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x=start_x, y=start_y;
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
                else if (e.type == SDL_MOUSEMOTION)
                {
                    SDL_GetMouseState(&x, &y);
                    std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                }
                if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    SDL_GetMouseState(&x, &y);
                    
                    float x_world = x;// * scale_factor;
                   
                    float y_world = y; //* scale_factor;


                    goal_state << x_world, y_world, 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                }
                    if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p) {
                        std::thread p(rysuj, x_history, y_history, theta_history, time);
                        p.detach();
                    }
                }

            }

            SDL_Delay((int)(dt * 1000));

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);

            // Store trajectory history
            state = quadrotor.GetState();
            x_history.push_back(state(0));
            y_history.push_back(state(1));
            theta_history.push_back(state(2));
        }

    }
    // Zwalnianie zasob�w SDL
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
