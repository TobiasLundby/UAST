#include "Joystick.hpp"

#include <sigc++/signal.h>
#include <sigc++/connection.h>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <string>
#include <mutex>
#include <linux/fs.h>

Joystick::Joystick(const std::string &_filename, int _id)
{
        init = false;
        id = _id;
        filename = _filename;
        if ((file = open(filename.c_str(), O_RDONLY)) < 0){
                printf("Missing device: %s \n", filename.c_str());
        }
        else{
                attach(filename);
        }
        setAxisDeadzone(8000);
}

Joystick::Joystick(int _id)
{
        init = false;
        id = _id;
        setAxisDeadzone(8000);
}

Joystick::~Joystick()
{
        close(file);
}

void Joystick::attach(const std::string &_filename)
{
        filename = _filename;
        if ((file = open(filename.c_str(), O_RDONLY)) < 0){
                printf("Missing device: %s \n", filename.c_str());
        }
        else{
                ioctl(file, JSIOCGAXES,    &axisCount);
                ioctl(file, JSIOCGBUTTONS, &buttonCount);
                ioctl(file, JSIOCGNAME(sizeof(name)), name);

                buttonStateMutex = new std::mutex[buttonCount];
                buttonState = new bool[buttonCount];
                axisStateMutex = new std::mutex[axisCount];
                axisState = new int[axisCount];
                axisReference = new int[axisCount];
                axisDeadzone = new int[axisCount];

                pressed = new (void(*[18])(Joystick*, int));
                released = new (void(*[18])(Joystick*, int));
                moved = new (void(*[8])(Joystick*, int, int));

                if (updateThread != NULL) delete updateThread;
                updateThread = new std::thread(updateJoystick, this, id);
		init = true;
        }
}

void Joystick::setAxisDeadzone(int deadzone)
{
        for (int i = 0; i < axisCount; i++){
                axisDeadzone[i] = deadzone;
        }
}

void Joystick::setAxisDeadzone(int id, int deadzone)
{
        axisDeadzone[id] = deadzone;
}

void Joystick::calibrate()
{
        for (int i = 0; i < axisCount; i++){
                axisReference[i] = axisState[i];
        }
}

void Joystick::setAxisReference(int id, int reference)
{
        axisReference[id] = reference;
}

bool Joystick::getButtonState(int id)
{
        buttonStateMutex[id].lock();
        bool state = buttonState[id];
        buttonStateMutex[id].unlock();
        return state;
}

int Joystick::getAxisState(int id)
{
        axisStateMutex[id].lock();
        int state = axisState[id];
        axisStateMutex[id].unlock();
        return state;
}

void Joystick::reset()
{
        for (int i = 0; i < buttonCount; i++){
                buttonStateMutex[i].lock();
                buttonState[i] = false;
                buttonStateMutex[i].unlock();
        }
        for (int i = 0; i < axisCount; i++){
                axisStateMutex[i].lock();
                axisState[i] = false;
                axisStateMutex[i].unlock();
        }
}

void Joystick::addPressedCall(int id, void (*function)(Joystick*, int))
{
        pressed[id] = function;
}

void Joystick::addReleasedCall(int id, void (*function)(Joystick*, int))
{
        released[id] = function;
}

void Joystick::addMovedCall(int id, void (*function)(Joystick*, int, int))
{
        moved[id] = function;
}

void Joystick::removePressedCall(int id)
{
        pressed[id] = NULL;
}

void Joystick::removeReleasedCall(int id)
{
        released[id] = NULL;
}

void Joystick::removeMovedCall(int id)
{
        moved[id] = NULL;
}

std::string Joystick::getName()
{
        return std::string(name);
}

void updateJoystick(Joystick *js, int id)
{
        while (true){
                if (js->init){
                        struct js_event event;

                        read(js->file, &event, sizeof(event));

                        if (event.type & JS_EVENT_AXIS){
                                js->axisStateMutex[event.number].lock();
                                js->axisState[event.number] = event.value - js->axisReference[event.number];
                                if (js->axisState[event.number] > -js->axisDeadzone[event.number] && js->axisState[event.number] < js->axisDeadzone[event.number]) js->axisState[event.number] = 0;
                                js->axisStateMutex[event.number].unlock();
                                if (js->moved[event.number] != NULL) js->moved[event.number](js, id, event.value);
                        }

                        else if (event.type & JS_EVENT_BUTTON){
                                js->buttonStateMutex[event.number].lock();
                                js->buttonState[event.number] = event.value;
                                js->buttonStateMutex[event.number].unlock();
                                if (event.value && js->pressed[event.number] != NULL) js->pressed[event.number](js, id);
                                if (!event.value && js->released[event.number] != NULL) js->released[event.number](js, id);
                        }
                }
        }
}
