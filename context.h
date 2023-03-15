//
// Created by orw on 3/14/23.
//
#include "subscriber.h"

#ifndef CODECRAFTSDK_CONTEXT_H
#define CODECRAFTSDK_CONTEXT_H

class Context{
public:
    Context();
    void run();
private:
    Subscriber subscriber;
};

#endif //CODECRAFTSDK_CONTEXT_H
