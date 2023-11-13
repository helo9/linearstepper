namespace result {
    template<class T, class Err>
    class Result {
    public:
        Result(T val) : is_ok(true), _val(val) {};
        Result(Err err) : is_ok(false), _err(err) {};

        const bool is_ok;

        inline T value() const {
            return _val;
        }

        inline Err error() const {
            return _err;
        }

        static inline Result Ok(T _val) {
            return Result(_val);
        }

        static inline Result Error(Err _err) {
            return Result(_err);
        }

    private:
        union {
            T _val;
            Err _err;
        };
    };
}
