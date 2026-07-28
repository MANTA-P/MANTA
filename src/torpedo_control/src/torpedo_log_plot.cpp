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

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream(line);
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

double parse_number(const std::string & value)
{
  if (value.empty()) {
    return kNaN;
  }
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return kNaN;
  }
}

std::string html_escape(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '&':
        escaped += "&amp;";
        break;
      case '<':
        escaped += "&lt;";
        break;
      case '>':
        escaped += "&gt;";
        break;
      case '"':
        escaped += "&quot;";
        break;
      default:
        escaped += character;
        break;
    }
  }
  return escaped;
}

struct CsvTable
{
  std::unordered_map<std::string, std::vector<double>> numeric_columns;
  std::vector<std::pair<double, std::string>> events;
  std::size_t row_count{0};

  const std::vector<double> * column(const std::string & name) const
  {
    const auto iterator = numeric_columns.find(name);
    if (iterator == numeric_columns.end()) {
      return nullptr;
    }
    return &iterator->second;
  }
};

CsvTable load_csv(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("Cannot open CSV file: " + path.string());
  }

  std::string header_line;
  if (!std::getline(input, header_line)) {
    throw std::runtime_error("CSV file is empty: " + path.string());
  }

  const auto headers = split_csv_line(header_line);
  std::unordered_map<std::string, std::size_t> indexes;
  for (std::size_t index = 0; index < headers.size(); ++index) {
    indexes[headers[index]] = index;
  }

  const auto elapsed_iterator = indexes.find("elapsed_sec");
  if (elapsed_iterator == indexes.end()) {
    throw std::runtime_error("CSV is missing required elapsed_sec column");
  }

  const auto event_iterator = indexes.find("key_events");
  CsvTable table;
  for (const auto & header : headers) {
    if (header != "key_events") {
      table.numeric_columns.emplace(header, std::vector<double>{});
    }
  }

  std::string line;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = split_csv_line(line);

    for (std::size_t index = 0; index < headers.size(); ++index) {
      if (headers[index] == "key_events") {
        continue;
      }
      const double value = index < fields.size() ? parse_number(fields[index]) : kNaN;
      table.numeric_columns[headers[index]].push_back(value);
    }

    if (event_iterator != indexes.end() && event_iterator->second < fields.size()) {
      const std::string & event = fields[event_iterator->second];
      if (!event.empty() && event != "NONE") {
        const double elapsed = elapsed_iterator->second < fields.size() ?
          parse_number(fields[elapsed_iterator->second]) : kNaN;
        table.events.emplace_back(elapsed, event);
      }
    }
    ++table.row_count;
  }

  if (table.row_count == 0U) {
    throw std::runtime_error("CSV has a header but no data rows");
  }
  return table;
}

struct Series
{
  std::string label;
  std::string column;
  std::string color;
};

bool has_finite_value(const std::vector<double> & values)
{
  return std::any_of(values.begin(), values.end(), [](const double value) {
              return std::isfinite(value);
  });
}

void draw_chart(
  std::ostream & output,
  const CsvTable & table,
  const std::string & title,
  const std::string & y_label,
  const std::vector<Series> & requested_series)
{
  const auto * time = table.column("elapsed_sec");
  if (time == nullptr || !has_finite_value(*time)) {
    output << "<p>Missing elapsed_sec data.</p>\n";
    return;
  }

  std::vector<std::pair<Series, const std::vector<double> *>> series;
  double y_min = std::numeric_limits<double>::infinity();
  double y_max = -std::numeric_limits<double>::infinity();
  for (const auto & requested : requested_series) {
    const auto * values = table.column(requested.column);
    if (values == nullptr || !has_finite_value(*values)) {
      continue;
    }
    series.emplace_back(requested, values);
    for (const double value : *values) {
      if (std::isfinite(value)) {
        y_min = std::min(y_min, value);
        y_max = std::max(y_max, value);
      }
    }
  }

  if (series.empty()) {
    output << "<section><h2>" << html_escape(title) << "</h2>"
           << "<p>No valid data received for this chart.</p></section>\n";
    return;
  }

  double x_min = std::numeric_limits<double>::infinity();
  double x_max = -std::numeric_limits<double>::infinity();
  for (const double value : *time) {
    if (std::isfinite(value)) {
      x_min = std::min(x_min, value);
      x_max = std::max(x_max, value);
    }
  }
  if (x_max <= x_min) {
    x_max = x_min + 1.0;
  }
  if (y_max <= y_min) {
    const double padding = std::max(1.0, std::abs(y_min) * 0.1);
    y_min -= padding;
    y_max += padding;
  } else {
    const double padding = 0.08 * (y_max - y_min);
    y_min -= padding;
    y_max += padding;
  }

  constexpr double width = 1200.0;
  constexpr double height = 330.0;
  constexpr double left = 85.0;
  constexpr double right = 25.0;
  constexpr double top = 35.0;
  constexpr double bottom = 55.0;
  const double plot_width = width - left - right;
  const double plot_height = height - top - bottom;

  const auto map_x = [&](const double value) {
      return left + (value - x_min) / (x_max - x_min) * plot_width;
    };
  const auto map_y = [&](const double value) {
      return top + (y_max - value) / (y_max - y_min) * plot_height;
    };

  output << "<section><h2>" << html_escape(title) << "</h2>\n";
  output << "<div class=\"legend\">";
  for (const auto & item : series) {
    output << "<span style=\"color:" << item.first.color << "\">&#9632; "
           << html_escape(item.first.label) << "</span> ";
  }
  output << "</div>\n";
  output << "<svg viewBox=\"0 0 " << width << ' ' << height
         << "\" role=\"img\" aria-label=\"" << html_escape(title) << "\">\n";
  output << "<rect x=\"" << left << "\" y=\"" << top << "\" width=\""
         << plot_width << "\" height=\"" << plot_height
         << "\" fill=\"white\" stroke=\"#555\"/>\n";

  output << std::fixed << std::setprecision(3);
  for (int grid = 0; grid <= 5; ++grid) {
    const double ratio = static_cast<double>(grid) / 5.0;
    const double x = left + ratio * plot_width;
    const double x_value = x_min + ratio * (x_max - x_min);
    output << "<line x1=\"" << x << "\" y1=\"" << top << "\" x2=\"" << x
           << "\" y2=\"" << top + plot_height
           << "\" stroke=\"#e5e7eb\"/>\n";
    output << "<text x=\"" << x << "\" y=\"" << height - 22.0
           << "\" text-anchor=\"middle\">" << x_value << "</text>\n";

    const double y = top + ratio * plot_height;
    const double y_value = y_max - ratio * (y_max - y_min);
    output << "<line x1=\"" << left << "\" y1=\"" << y << "\" x2=\""
           << left + plot_width << "\" y2=\"" << y
           << "\" stroke=\"#e5e7eb\"/>\n";
    output << "<text x=\"" << left - 10.0 << "\" y=\"" << y + 4.0
           << "\" text-anchor=\"end\">" << y_value << "</text>\n";
  }

  output << "<text x=\"" << left + plot_width / 2.0 << "\" y=\"" << height - 4.0
         << "\" text-anchor=\"middle\">elapsed time [s]</text>\n";
  output << "<text x=\"18\" y=\"" << top + plot_height / 2.0
         << "\" transform=\"rotate(-90 18 " << top + plot_height / 2.0
         << ")\" text-anchor=\"middle\">" << html_escape(y_label) << "</text>\n";

  const std::size_t point_limit = 5000U;
  const std::size_t stride = std::max<std::size_t>(1U, time->size() / point_limit);
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
      output << (drawing ? " L " : " M ") << map_x((*time)[index]) << ' '
             << map_y(values[index]);
      drawing = true;
    }
    output << "\" fill=\"none\" stroke=\"" << item.first.color
           << "\" stroke-width=\"1.7\"/>\n";
  }

  output << "</svg></section>\n";
}

void write_html(
  const CsvTable & table,
  const std::filesystem::path & csv_path,
  const std::filesystem::path & output_path)
{
  std::ofstream output(output_path);
  if (!output.is_open()) {
    throw std::runtime_error("Cannot create HTML file: " + output_path.string());
  }

  output << "<!doctype html><html><head><meta charset=\"utf-8\">"
         << "<title>Torpedo control log</title><style>"
         << "body{font-family:system-ui,sans-serif;margin:24px;background:#f3f4f6;color:#111827}"
         << "section{background:white;padding:14px 18px;margin:18px 0;border-radius:10px;"
         << "box-shadow:0 1px 4px #0002}svg{width:100%;height:auto}"
         << ".legend span{margin-right:16px}table{border-collapse:collapse;width:100%}"
         << "th,td{border-bottom:1px solid #ddd;padding:5px;text-align:left}"
         << "text{font-size:12px;fill:#374151}</style></head><body>\n";
  output << "<h1>Torpedo control log</h1><p>Source: <code>"
         << html_escape(csv_path.string()) << "</code><br>Rows: " << table.row_count << "</p>\n";

  draw_chart(output, table, "Thrust command", "command", {
      {"thrust", "thrust_command", "#111827"}});
  draw_chart(output, table, "Fin commands", "angle [rad]", {
      {"top", "fin_top_command_rad", "#dc2626"},
      {"bottom", "fin_bottom_command_rad", "#ea580c"},
      {"left", "fin_left_command_rad", "#2563eb"},
      {"right", "fin_right_command_rad", "#16a34a"}});
  draw_chart(output, table, "Body linear velocity from odometry", "velocity [m/s]", {
      {"X right", "odom_twist_linear_x", "#dc2626"},
      {"Y forward", "odom_twist_linear_y", "#16a34a"},
      {"Z up", "odom_twist_linear_z", "#2563eb"}});
  draw_chart(output, table, "Body angular velocity from odometry", "rate [rad/s]", {
      {"X pitch", "odom_twist_angular_x", "#dc2626"},
      {"Y roll", "odom_twist_angular_y", "#16a34a"},
      {"Z yaw", "odom_twist_angular_z", "#2563eb"}});
  draw_chart(output, table, "World position", "position [m]", {
      {"X", "odom_position_x", "#dc2626"},
      {"Y", "odom_position_y", "#16a34a"},
      {"Z", "odom_position_z", "#2563eb"}});
      
  // [신규 추가] 타겟 위치 및 추적 오차 그래프 그룹화 적용
  const bool has_target_data = 
    table.column("target_x") != nullptr || 
    table.column("error_pitch_rad") != nullptr || 
    table.column("auto_track_mode") != nullptr;

  if (has_target_data) {
    draw_chart(output, table, "Target World Position", "position [m]", {
        {"target_x", "target_x", "#dc2626"},
        {"target_y", "target_y", "#16a34a"},
        {"target_z", "target_z", "#2563eb"}});
    
    draw_chart(output, table, "Tracking Error Angles", "angle [rad]", {
        {"Pitch (X-axis) error", "error_pitch_rad", "#dc2626"},
        {"Yaw (Z-axis) error", "error_yaw_rad", "#2563eb"}});

    draw_chart(output, table, "Auto Track Mode", "status (1=ON)", {
        {"Auto Track ON/OFF", "auto_track_mode", "#4b5563"}});
  }

  draw_chart(output, table, "Measured fin joint positions", "angle [rad]", {
      {"top", "joint_fin_top_position_rad", "#dc2626"},
      {"bottom", "joint_fin_bottom_position_rad", "#ea580c"},
      {"left", "joint_fin_left_position_rad", "#2563eb"},
      {"right", "joint_fin_right_position_rad", "#16a34a"}});

  const bool has_propeller_joint_data =
    table.column("joint_propeller_front_position_rad") != nullptr ||
    table.column("joint_propeller_front_velocity_rad_per_sec") != nullptr ||
    table.column("joint_propeller_rear_position_rad") != nullptr ||
    table.column("joint_propeller_rear_velocity_rad_per_sec") != nullptr;
  if (has_propeller_joint_data) {
    draw_chart(output, table, "Measured propeller joint positions", "angle [rad]", {
        {"front", "joint_propeller_front_position_rad", "#7c3aed"},
        {"rear", "joint_propeller_rear_position_rad", "#0891b2"}});
    draw_chart(output, table, "Measured propeller joint velocities", "rate [rad/s]", {
        {"front", "joint_propeller_front_velocity_rad_per_sec", "#7c3aed"},
        {"rear", "joint_propeller_rear_velocity_rad_per_sec", "#0891b2"}});
  }

  output << "<section><h2>Input events</h2><table><thead><tr><th>Elapsed [s]</th>"
         << "<th>Events</th></tr></thead><tbody>\n";
  const std::size_t event_limit = 1000U;
  const std::size_t count = std::min(event_limit, table.events.size());
  output << std::fixed << std::setprecision(3);
  for (std::size_t index = 0; index < count; ++index) {
    output << "<tr><td>" << table.events[index].first << "</td><td>"
           << html_escape(table.events[index].second) << "</td></tr>\n";
  }
  if (table.events.size() > event_limit) {
    output << "<tr><td colspan=\"2\">Only the first " << event_limit
           << " events are shown.</td></tr>\n";
  }
  output << "</tbody></table></section></body></html>\n";
}
}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 2 || argc > 3) {
    std::cerr << "Usage: " << argv[0] << " <torpedo_session.csv> [output.html]\n";
    return 2;
  }

  try {
    const std::filesystem::path csv_path(argv[1]);
    std::filesystem::path output_path;
    if (argc == 3) {
      output_path = argv[2];
    } else {
      output_path = csv_path.parent_path() / (csv_path.stem().string() + "_plot.html");
    }

    const CsvTable table = load_csv(csv_path);
    write_html(table, csv_path, output_path);
    std::cout << "Wrote " << output_path << '\n';
  } catch (const std::exception & error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
  return 0;
}