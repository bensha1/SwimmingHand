#ifndef STATE_HISTORY_H
#define STATE_HISTORY_H

class StateHistory {
    public:
        int buff_size;
        int i;
        int* state_buff;
        bool was_buff_filled;
        StateHistory(int buff_size) : buff_size(buff_size), i(0), was_buff_filled(false) {
            state_buff = new int[buff_size]; }

        ~StateHistory() {
            delete[] state_buff;
        }

        void update_state(int new_state) {
            state_buff[i] = new_state;
            if (!was_buff_filled && i == buff_size - 1) {
                was_buff_filled = true;
            }
            i = (i + 1) % buff_size;
        }

        bool compare_lower_half(int state) {
            if (!was_buff_filled) return false;

            return compare_internal(i, state);
        }

        bool compare_higher_half(int state) {
            if (!was_buff_filled) return false;
            
            int base_i = (i - (buff_size / 2)) % buff_size;
            if (base_i < 0) base_i = -base_i;
            return compare_internal(base_i, state);
        }

        bool compare_internal(int base_i, int state) {
            int i_counter = 0;
            int j = base_i;
            while (i_counter < buff_size / 2) {
                if (state_buff[j] != state) {
                    return false;
                }
                j = (j + 1) % buff_size;
                i_counter++;
            }
            return true;
        }

        int get_prev() {
            int prev_i = (i - 1) % buff_size;
            return state_buff[prev_i];
        }
};

#endif
