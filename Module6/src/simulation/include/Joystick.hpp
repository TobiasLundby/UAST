#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <string>
#include <mutex>
#include <thread>

//////////
// Xbox //
//////////
#define XBOX_One_Controller             "Microsoft X-Box One pad"
// Axis
#define XBOX_Left_Analog_Horizontal      0
#define XBOX_Left_Analog_Vertical        1
#define XBOX_Left_Trigger                2
#define XBOX_Right_Analog_Horizontal     3
#define XBOX_Right_Analog_Vertical       4
#define XBOX_Right_Trigger               5
#define XBOX_Horizontal_D_Pad            6
#define XBOX_Vertical_D_Pad              7
// Buttons
#define XBOX_A                           0
#define XBOX_B                           1
#define XBOX_X                           2
#define XBOX_Y                           3
#define XBOX_Left_Bumper                 4
#define XBOX_Right_Bumper                5
#define XBOX_BACK                        6
#define XBOX_START                       7
#define XBOX_LOGO                        8
#define XBOX_Left_Analog                 9
#define XBOX_Right_Analog               10

/////////
// G25 //
/////////
#define G25_Controller                  ""
// Axis
#define G25_Wheel                        0
#define G25_Coupling                     1
#define G25_Gas                          2
#define G25_Break                        3
#define G25_Horizontal_D_Pad             4
#define G25_Vertical_D_Pad               5
// Buttons
#define G25_Button_Down                  0
#define G25_Button_Left                  1
#define G25_Button_Right                 2
#define G25_Button_Up                    3
#define G25_Sequential_Left              4
#define G25_Sequential_Right             5
#define G25_Wheel_Button_Right           6
#define G25_Wheel_Button_Left            7
#define G25_Button_2                     8
#define G25_Button_3                     9
#define G25_Button_4                    10
#define G25_Button_1                    11
#define G25_Gear_Up                     12
#define G25_Gear_Down                   13
#define G25_Gear_1                      12
#define G25_Gear_2                      13
#define G25_Gear_3                      14
#define G25_Gear_4                      15
#define G25_Gear_5                      16
#define G25_Gear_6                      17
#define G25_Gear_R                      18

class Joystick{

        public:

                bool init;

                int id;

                std::string filename;

                int file;
                char name[128];

                int buttonCount;
                int axisCount;

                std::mutex *buttonStateMutex;
                bool *buttonState;
                std::mutex *axisStateMutex;
                int *axisState;
                int *axisReference;
                int *axisDeadzone;

                void (**pressed)(Joystick*, int);
                void (**released)(Joystick*, int);
                void (**moved)(Joystick*, int val, int);

                Joystick(const std::string &filename, int id);
                Joystick(int id);
                ~Joystick();

                void attach(const std::string &filename);
                void attach();

                void setAxisDeadzone(int deadzone);
                void setAxisDeadzone(int id, int deadzone);
                void setAxisReference(int id, int reference);

                void calibrate();

                void reset();

                bool getButtonState(int id);
                int getAxisState(int id);

                void addPressedCall(int id, void (*function)(Joystick*, int));
                void addReleasedCall(int id, void (*function)(Joystick*, int));
                void addMovedCall(int id, void (*function)(Joystick*, int, int));

                void removePressedCall(int id);
                void removeReleasedCall(int id);
                void removeMovedCall(int id);

                std::string getName();

        private:

                std::thread *updateThread;
};

void updateJoystick(Joystick *js, int id);

#endif
