#pragma once

#include <filesystem>

namespace bluerov_integration::team_byung
{

void generatePpidHtml(
  const std::filesystem::path & csv_path,
  const std::filesystem::path & html_path);

}  // namespace bluerov_integration::team_byung
