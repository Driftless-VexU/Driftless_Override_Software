#include "driftless/MatchControllerFactory.hpp"

// General includes
#include "driftless/menu/MenuAdapter.hpp"
#include "driftless/pros_adapters/ProsClock.hpp"
#include "driftless/pros_adapters/ProsDelayer.hpp"

// alliance includes
#include "driftless/alliance/BlueAlliance.hpp"
#include "driftless/alliance/RedAlliance.hpp"
#include "driftless/alliance/SkillsAlliance.hpp"

// auton includes
#include "driftless/auton/BlueMatchAuton.hpp"
#include "driftless/auton/OrangeMatchAuton.hpp"
#include "driftless/auton/OrangeSafeMatchAuton.hpp"

// config includes
#include "driftless/config/BlueConfig.hpp"
#include "driftless/config/OrangeConfig.hpp"

// profile includes
#include "driftless/profiles/AsherProfile.hpp"
#include "driftless/profiles/AsherSkillsProfile.hpp"
#include "driftless/profiles/KadenProfile.hpp"
#include "driftless/profiles/KadenSkillsProfile.hpp"

namespace driftless {
driftless::MatchController MatchControllerFactory::createMatchController() {
  // the display menu
  std::unique_ptr<menu::IMenu> lvgl_menu{std::make_unique<menu::MenuAdapter>()};
  // add alliances
  std::shared_ptr<alliance::IAlliance> blue_alliance{
      std::make_shared<alliance::BlueAlliance>()};
  lvgl_menu->addAlliance(blue_alliance);

  std::shared_ptr<alliance::IAlliance> red_alliance{
      std::make_shared<alliance::RedAlliance>()};
  lvgl_menu->addAlliance(red_alliance);

  std::shared_ptr<alliance::IAlliance> skills_alliance{
      std::make_shared<alliance::SkillsAlliance>()};
  lvgl_menu->addAlliance(skills_alliance);

  // add auton routes
  std::unique_ptr<auton::AAuton> blue_match_auton{
      std::make_unique<auton::BlueMatchAuton>()};
  lvgl_menu->addAuton((std::move(blue_match_auton)));
  std::unique_ptr<auton::AAuton> orange_match_auton{
      std::make_unique<auton::OrangeMatchAuton>()};
  lvgl_menu->addAuton(std::move(orange_match_auton));
  std::unique_ptr<auton::AAuton> orange_safe_match_auton{
      std::make_unique<auton::OrangeSafeMatchAuton>()};
  lvgl_menu->addAuton(std::move(orange_safe_match_auton));

  // add configs
  std::unique_ptr<config::IConfig> blue_config{
      std::make_unique<config::BlueConfig>()};
  lvgl_menu->addConfig(std::move(blue_config));
  std::unique_ptr<config::IConfig> orange_config{
      std::make_unique<config::OrangeConfig>()};
  lvgl_menu->addConfig(std::move(orange_config));

  // add profiles
  std::unique_ptr<profiles::IProfile> kaden_profile{
      std::make_unique<driftless::profiles::KadenProfile>()};
  lvgl_menu->addProfile(std::move(kaden_profile));
  std::unique_ptr<profiles::IProfile> kaden_skills_profile{
      std::make_unique<profiles::KadenSkillsProfile>()};
  lvgl_menu->addProfile(std::move(kaden_skills_profile));

  std::unique_ptr<profiles::IProfile> asher_profile{
      std::make_unique<profiles::AsherProfile>()};
  lvgl_menu->addProfile(std::move(asher_profile));
  std::unique_ptr<profiles::IProfile> asher_skills_profile{
      std::make_unique<profiles::AsherSkillsProfile>()};
  lvgl_menu->addProfile(std::move(asher_skills_profile));

  // create RTOS
  std::shared_ptr<rtos::IClock> clock{
      std::make_unique<pros_adapters::ProsClock>()};
  std::unique_ptr<rtos::IDelayer> delayer{
      std::make_unique<pros_adapters::ProsDelayer>()};

  // create and send out the match controller
  return MatchController{std::move(lvgl_menu), clock, std::move(delayer)};
}
}  // namespace driftless