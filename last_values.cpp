
#include <iostream>
#include <chrono>
#include <unistd.h>

using namespace std::chrono;

template <size_t N>
struct last_values {
  high_resolution_clock::time_point last_time;
  uint8_t last_data[N];
};

template <size_t N>
bool send_req(uint8_t (&data)[N], last_values<N> &l_data) {
  if (duration_cast<milliseconds>(high_resolution_clock::now() - l_data.last_time).count() > 100) {				// prüft ob das letzte Mal senden länger als 100ms her ist
    std::copy(std::begin(data), std::end(data), std::begin(l_data.last_data));				// kopiert data in last_data.last_data
		l_data.last_time = high_resolution_clock::now();							// aktualisiert den time_point des letzten Sendens
    std::cout << "Last data refresh is longer ago than 100ms, sending anyway. ";
		return true;
  }
  else if (!std::equal(std::begin(data), std::end(data), std::begin(l_data.last_data))) {			// prüft ob die Daten array gleich sind
    std::copy(std::begin(data), std::end(data), std::begin(l_data.last_data));
		l_data.last_time = high_resolution_clock::now();							// aktualisiert den time_point des letzten Sendens
    std::cout << "Data isn't equal, save data to last_data and send it ";
		return true;
  }
  std::cout << "Data is equal, do not send it " ;
  return false;
}

last_values<2> last_state;
uint8_t state_data[2];

int setMotorState(uint8_t side, uint8_t state) {
  state_data[0] = side;
  state_data[1] = state;

  std::cout << duration_cast<milliseconds>(high_resolution_clock::now() - last_state.last_time).count() << '\n';

  std::cout << send_req(state_data, last_state) << std::endl;

  return 1;
}


int main()  {
  last_state.last_time = high_resolution_clock::now();
  setMotorState(1,5);
  usleep(10 * 1000);
  setMotorState(1,6);
  usleep(99 * 1000);
  setMotorState(1,6);
  usleep(10 * 1000);
  setMotorState(1,6);

  return 1;
}
