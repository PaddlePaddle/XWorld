"""
This file contains some common python utility functions.
"""

def overrides(interface_class):
    """
    This function is used for specifying a function in the derived class
    that is overriding an inherited class method.
    """
    def overrider(method):
        assert(method.__name__ in dir(interface_class))
        return method
    return overrider
