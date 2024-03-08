#pragma once

#include <fstream>
#include <string>
#include <string_view>
#include <expected>

namespace nhk24_2nd_ws::file_loader {
	/**
	 * @brief ファイルを読み込む
	 * 
	 * @param path ファイルのパス
	 * @return std::expected<std::string, std::string> 成功時はファイルの内容、失敗時はエラーメッセージ
	 */
	inline auto load_file(const std::string_view path) -> std::expected<std::u8string, std::string> {
		using namespace std::string_literals;
		std::basic_ifstream<char> ifs(path.data());
		if(!ifs) {
			return std::unexpected{"Failed to open file: "s + std::string(path)};
		}

		const auto ret_ = std::string(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
		const auto p = reinterpret_cast<const char8_t *>(ret_.c_str());
		std::u8string ret(p, p + ret_.size());
		return {std::move(ret)};
	}
}