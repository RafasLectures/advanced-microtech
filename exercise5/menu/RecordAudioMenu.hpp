#ifndef ADVANVED_MICROTECH_RECORDAUDIOMENU_HPP
#define ADVANVED_MICROTECH_RECORDAUDIOMENU_HPP

#include "Menu.hpp"
namespace AdvancedMicrotech {

class RecordAudioMenu : public Menu<RecordAudioMenu> {
public:
  static constexpr uint8_t X_POS = 1;
  static constexpr uint8_t Y_POS = 1;

  static constexpr const char* getTitle() {
    return "Play";
  }
  static constexpr const char* getMenuText() {
    return "Recording       \n\x02Stop Recording ";
  }

  static void clearTitle();

  static void showItems();

  static void specificUpAction();
  static void specificDownAction();
  static void specificLeftAction();
  static void specificRightAction();
  static void specificEnterAction();

  static void recordingFinishedCallback();

private:
};

}  // namespace AdvancedMicrotech

#endif  // ADVANVED_MICROTECH_RECORDAUDIOMENU_HPP
