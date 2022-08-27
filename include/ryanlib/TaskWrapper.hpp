#pragma once
#include "pros/rtos.hpp"
#include <memory>

namespace ryan{

/*
 * A utility class that wraps a task trampoline. To use, simply inherit your class from TaskWrapper
 * and override the `loop` method. To start the task, the `startTask` method must be called, either
 * from the constructor or from outside the class. Credit for initial idea / implementation goes to 
 * Theo Lemay (7842F)
 * 
 */

class TaskWrapper {
    protected:
    /**
     * @brief Construct a new Task Wrapper object
     * 
     */
    TaskWrapper() = default;

    /**
     * @brief Copying between tasks is not allowed
     */
    TaskWrapper(const TaskWrapper& iTask) = delete;

    /**
     * @brief Construct a new Task Wrapper object
     * 
     * @param iTask rvalue of a task
     */
    TaskWrapper(TaskWrapper&& iTask) = default;

    /**
     * @brief Destroy the Task Wrapper object, virtual to allow inheritance
     * 
     */
    virtual ~TaskWrapper();

    /**
     * @brief the task loop to run
     * Override this function to implement a custom task loop.
     */
    virtual void loop() = 0;

    public:
    /**   
     * @brief Start the task.
     *
     * @param iname The task name, optional.
     */
    virtual void startTask(const char* iname = "task");

    /**
     * @brief resumes the task.
     */
    virtual void resumeTask();

    /**
     * @brief pauses the task.
     */
    virtual void pauseTask();

    /**
     * @brief kills the task.
     */
    virtual void stopTask();
    
    /**
     * @brief Get the task name.
     *
     * @return The name.
     */
    virtual const char* getName();

    private:
        static void trampoline(void* iparam);
        std::unique_ptr<pros::Task> task {nullptr};
};
}