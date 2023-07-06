#include "diagnostic.h"

// singleton instance
Diagnostic *Diagnostic::_singleton;

Diagnostic &diagnostic()
{
    return *Diagnostic::get_singleton();
}
