"""
The c-lang-integers module provides integer types behaving similar to
integers in c code.
"""


class CUInt:
    "A unsigned int with limited bitwith and overflow"

    def __init__(self, initial_value: int, bytewidth: int = 4):

        if isinstance(initial_value, int):
            self._bytewidth = bytewidth
            self._value = initial_value & self.bitmask(8 * self._bytewidth)

        elif isinstance(initial_value, CUInt):
            self._bytewidth = initial_value._bytewidth
            self._value = initial_value._value

        else:
            raise TypeError(f"Cannot initalize CUInt with type {type(initial_value)} as `initial_value`")

    @property
    def value(self):
        return self._value

    def __mul__(self, other):
        if isinstance(other, CUInt):

            return CUInt(self._value * other._value, self._bytewidth)

        elif isinstance(other, int):

            return CUInt(self._value * other, self._bytewidth)

        raise NotImplementedError(f"Multiplication with type {type(other)} is not implemented fo CInt")

    def __add__(self, other):
        if isinstance(other, CUInt):

            return CUInt(self._value + other._value, self._bytewidth)

        elif isinstance(other, int):

            return CUInt(self._value + other, self._bytewidth)

    def __sub__(self, other):
        if isinstance(other, CUInt):

            return CUInt(self._value - other._value, self._bytewidth)

        elif isinstance(other, int):

            return CUInt(self._value - other, self._bytewidth)

    def __truediv__(self, other):
        if isinstance(other, CUInt):

            return CUInt(self._value // other._value, self._bytewidth)

        elif isinstance(other, int):

            return CUInt(self._value // other, self._bytewidth)

    def _truncate_exceed_bits(self):
        self._value = self._value & self.bitmask(8 * self._bytewidth)

    def __repr__(self):
        return f"CInt({self._value},{self._bytewidth})"

    @staticmethod
    def bitmask(width: int):
        return 2**width - 1

def UInt32(value):
    return CUInt(value, 4)

def UInt16(value):
    return CUInt(value, 2)

def UInt8(value):
    return CUInt(value, 1)
