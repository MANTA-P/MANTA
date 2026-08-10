#include "bluerov_integration/team_byung/ppid_log_plot.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bluerov_integration::team_byung
{
namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> fields;
  std::istringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

double number(const std::string & text)
{
  try {
    return text.empty() ? kNaN : std::stod(text);
  } catch (const std::exception &) {
    return kNaN;
  }
}

std::string escapeHtml(const std::string & text)
{
  std::string result;
  for (const char character : text) {
    switch (character) {
      case '&': result += "&amp;"; break;
      case '<': result += "&lt;"; break;
      case '>': result += "&gt;"; break;
      case '"': result += "&quot;"; break;
      default: result += character; break;
    }
  }
  return result;
}

struct Table
{
  std::unordered_map<std::string, std::vector<double>> columns;
  std::vector<std::pair<double, std::string>> events;
  std::size_t rows{0};

  const std::vector<double> * column(const std::string & name) const
  {
    const auto found = columns.find(name);
    return found == columns.end() ? nullptr : &found->second;
  }
};

Table loadCsv(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Cannot open CSV: " + path.string());
  }
  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("CSV is empty");
  }
  const auto headers = splitCsv(line);
  const auto elapsed_it =
    std::find(headers.begin(), headers.end(), "elapsed_sec");
  if (elapsed_it == headers.end()) {
    throw std::runtime_error("CSV has no elapsed_sec column");
  }
  const std::size_t elapsed_index =
    static_cast<std::size_t>(std::distance(headers.begin(), elapsed_it));
  const auto event_it = std::find(headers.begin(), headers.end(), "event");
  const std::size_t event_index = event_it == headers.end() ?
    headers.size() :
    static_cast<std::size_t>(std::distance(headers.begin(), event_it));

  Table table;
  for (const auto & header : headers) {
    if (header != "event") {
      table.columns.emplace(header, std::vector<double>{});
    }
  }
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = splitCsv(line);
    for (std::size_t index = 0; index < headers.size(); ++index) {
      if (index == event_index) {
        continue;
      }
      table.columns[headers[index]].push_back(
        index < fields.size() ? number(fields[index]) : kNaN);
    }
    if (event_index < fields.size() && fields[event_index] != "NONE" &&
      !fields[event_index].empty())
    {
      table.events.emplace_back(
        elapsed_index < fields.size() ? number(fields[elapsed_index]) : kNaN,
        fields[event_index]);
    }
    ++table.rows;
  }
  if (table.rows == 0U) {
    throw std::runtime_error("CSV has no data rows");
  }
  return table;
}

struct Series
{
  std::string label;
  std::string column;
  std::string color;
};

void writeChart(
  std::ostream & output,
  const Table & table,
  const std::string & title,
  const std::string & unit,
  const std::vector<Series> & requested)
{
  const auto * time = table.column("elapsed_sec");
  if (time == nullptr || time->empty()) {
    return;
  }

  std::vector<std::pair<Series, const std::vector<double> *>> series;
  double x_min = std::numeric_limits<double>::infinity();
  double x_max = -x_min;
  double y_min = x_min;
  double y_max = -x_min;
  for (const double value : *time) {
    if (std::isfinite(value)) {
      x_min = std::min(x_min, value);
      x_max = std::max(x_max, value);
    }
  }
  for (const auto & item : requested) {
    const auto * values = table.column(item.column);
    if (values == nullptr) {
      continue;
    }
    bool has_data = false;
    for (const double value : *values) {
      if (std::isfinite(value)) {
        has_data = true;
        y_min = std::min(y_min, value);
        y_max = std::max(y_max, value);
      }
    }
    if (has_data) {
      series.emplace_back(item, values);
    }
  }
  if (series.empty() || !std::isfinite(x_min)) {
    return;
  }
  if (x_max <= x_min) {
    x_max = x_min + 1.0;
  }
  if (y_max <= y_min) {
    const double padding = std::max(1.0, std::abs(y_min) * 0.1);
    y_min -= padding;
    y_max += padding;
  } else {
    const double padding = (y_max - y_min) * 0.08;
    y_min -= padding;
    y_max += padding;
  }

  constexpr double width = 1200.0;
  constexpr double height = 340.0;
  constexpr double left = 85.0;
  constexpr double top = 30.0;
  constexpr double plot_width = 1090.0;
  constexpr double plot_height = 250.0;
  const auto map_x = [=](const double value) {
      return left + (value - x_min) / (x_max - x_min) * plot_width;
    };
  const auto map_y = [=](const double value) {
      return top + (y_max - value) / (y_max - y_min) * plot_height;
    };

  output << "<section><h2>" << escapeHtml(title) << "</h2><div class=\"legend\">";
  for (const auto & item : series) {
    output << "<span style=\"color:" << item.first.color << "\">&#9632; " <<
      escapeHtml(item.first.label) << "</span>";
  }
  output << "</div><svg viewBox=\"0 0 " << width << ' ' << height << "\">"
         << "<rect x=\"" << left << "\" y=\"" << top << "\" width=\"" <<
    plot_width << "\" height=\"" << plot_height <<
    "\" fill=\"white\" stroke=\"#475569\"/>";

  output << std::fixed << std::setprecision(3);
  for (int grid = 0; grid <= 5; ++grid) {
    const double ratio = static_cast<double>(grid) / 5.0;
    const double x = left + ratio * plot_width;
    const double y = top + ratio * plot_height;
    output << "<line x1=\"" << x << "\" y1=\"" << top << "\" x2=\"" <<
      x << "\" y2=\"" << top + plot_height << "\" stroke=\"#e2e8f0\"/>"
           << "<line x1=\"" << left << "\" y1=\"" << y << "\" x2=\"" <<
      left + plot_width << "\" y2=\"" << y << "\" stroke=\"#e2e8f0\"/>";
  }

  constexpr std::size_t point_limit = 5000U;
  const std::size_t stride = std::max<std::size_t>(
    1U, time->size() / point_limit);
  for (const auto & item : series) {
    const auto & values = *item.second;
    const std::size_t count = std::min(time->size(), values.size());
    output << "<path d=\"";
    bool drawing = false;
    for (std::size_t index = 0; index < count; index += stride) {
      if (!std::isfinite((*time)[index]) || !std::isfinite(values[index])) {
        drawing = false;
        continue;
      }
      output << (drawing ? " L " : " M ") << map_x((*time)[index]) << ' ' <<
        map_y(values[index]);
      drawing = true;
    }
    output << "\" fill=\"none\" stroke=\"" << item.first.color <<
      "\" stroke-width=\"1.7\"/>";
  }
  output << "<text x=\"600\" y=\"325\" text-anchor=\"middle\">time [s]</text>"
         << "<text x=\"18\" y=\"170\" transform=\"rotate(-90 18 170)\" "
         << "text-anchor=\"middle\">" << escapeHtml(unit) <<
    "</text></svg></section>";
}

void writeHtml(
  const Table & table,
  const std::filesystem::path & csv,
  const std::filesystem::path & html)
{
  std::ofstream output(html);
  if (!output) {
    throw std::runtime_error("Cannot create HTML: " + html.string());
  }
  output <<
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<title>BlueROV PPID log</title><style>"
    "body{font-family:system-ui;margin:24px;background:#f1f5f9;color:#0f172a}"
    "section{background:white;padding:14px 18px;margin:18px 0;border-radius:10px;"
    "box-shadow:0 1px 4px #0002}svg{width:100%;height:auto}"
    ".legend span{margin-right:16px}table{border-collapse:collapse;width:100%}"
    "th,td{padding:5px;border-bottom:1px solid #ddd}</style></head><body><h1>"
    "BlueROV PPID control log</h1><p>Source: <code>" <<
    escapeHtml(csv.string()) << "</code><br>Rows: " << table.rows << "</p>";

  writeChart(output, table, "World position", "position [m]", {
    {"target X", "target_position_x", "#f87171"},
    {"current X", "current_position_x", "#b91c1c"},
    {"target Y", "target_position_y", "#4ade80"},
    {"current Y", "current_position_y", "#15803d"},
    {"target Z", "target_position_z", "#60a5fa"},
    {"current Z", "current_position_z", "#1d4ed8"}});
  writeChart(output, table, "Position error", "error [m]", {
    {"X", "position_error_x", "#dc2626"},
    {"Y", "position_error_y", "#16a34a"},
    {"Z", "position_error_z", "#2563eb"}});
  writeChart(output, table, "Body velocity", "velocity [m/s]", {
    {"target X", "target_velocity_x", "#f87171"},
    {"current X", "current_velocity_x", "#b91c1c"},
    {"target Y", "target_velocity_y", "#4ade80"},
    {"current Y", "current_velocity_y", "#15803d"},
    {"target Z", "target_velocity_z", "#60a5fa"},
    {"current Z", "current_velocity_z", "#1d4ed8"}});
  writeChart(output, table, "Body command", "command", {
    {"X", "body_command_x", "#dc2626"},
    {"Y", "body_command_y", "#16a34a"},
    {"Z", "body_command_z", "#2563eb"},
    {"Yaw", "yaw_command", "#9333ea"}});
  writeChart(output, table, "Horizontal thrusters", "command", {
    {"M1", "motor1_command", "#dc2626"},
    {"M2", "motor2_command", "#ea580c"},
    {"M3", "motor3_command", "#16a34a"},
    {"M4", "motor4_command", "#2563eb"}});
  writeChart(output, table, "Vertical thrusters", "command", {
    {"M5", "motor5_command", "#7c3aed"},
    {"M6", "motor6_command", "#0891b2"}});

  output << "<section><h2>Target events</h2><table><tr><th>Time [s]</th>"
         << "<th>Event</th></tr>";
  for (const auto & event : table.events) {
    output << "<tr><td>" << event.first << "</td><td>" <<
      escapeHtml(event.second) << "</td></tr>";
  }
  output << "</table></section></body></html>";
}

}  // namespace

void generatePpidHtml(
  const std::filesystem::path & csv_path,
  const std::filesystem::path & html_path)
{
  writeHtml(loadCsv(csv_path), csv_path, html_path);
}

}  // namespace bluerov_integration::team_byung

#ifndef PPID_LOG_PLOT_LIBRARY
int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 3) {
    std::cerr << "Usage: ppid_log_plot <input.csv> [output.html]\n";
    return 2;
  }
  try {
    const std::filesystem::path csv = std::filesystem::absolute(argv[1]);
    const std::filesystem::path html = std::filesystem::absolute(
      argc == 3 ?
      std::filesystem::path(argv[2]) :
      csv.parent_path() / (csv.stem().string() + "_plot.html"));
    bluerov_integration::team_byung::generatePpidHtml(csv, html);
    std::cout << "Wrote: " << html << '\n';
  } catch (const std::exception & error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
#endif
