template<class T>
T clamp(T value, T lower_bound, T upper_bound) {
    if (value < lower_bound) {
        return lower_bound;
    } else if (value < upper_bound) {
        return value;
    } else {
        return upper_bound;
    }
}
