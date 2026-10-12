#include "driftless/robot/subsystems/lift/DirectContinuousLift.hpp"

namespace driftless::robot::subsystems::lift {
void DirectContinuousLift::init() {}

void DirectContinuousLift::run() {m_task->start(taskLoop,this);}

void DirectContinuousLift::setHeight(float height) {
    if (m_mutex){
        m_mutex->take();
    }

    m_height = height;

    if (m_mutex){
        m_mutex->give();
    }
}

float DirectContinuousLift::getHeight() {}

void DirectContinuousLift::taskUpdate() {
    if (m_mutex) {
        m_mutex->take();
    }
    
    


    if (m_mutex) {
        m_mutex->give();
    }

    m_delayer->delay(TASK_DELAY);
}

void DirectContinuousLift::taskLoop(void* args) {
    DirectContinuousLift* lift{static_cast<DirectContinuousLift*>(args)};

    while(1) {
        lift->taskUpdate();
    }
}
}