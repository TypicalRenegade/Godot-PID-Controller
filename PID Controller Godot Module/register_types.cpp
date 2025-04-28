/* register_types cpp */

// Code from godot docs

#include "register_types.h"

#include "core/object/class_db.h"
#include "pid.h"

void initialize_pid_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
            return;
    }
    ClassDB::register_class<PID>();
}

void uninitialize_pid_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
            return;
    }
   // Nothing to do here in this example.
}
