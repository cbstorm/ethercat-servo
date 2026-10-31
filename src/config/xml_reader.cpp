// SPDX-License-Identifier: MIT
#include "xml_reader.hpp"

#include <cctype>
#include <cstdlib>

namespace ethercat_motion::xml {

const std::string* Node::attribute(const std::string& key) const noexcept {
    for (const auto& [k, v] : attributes) {
        if (k == key) return &v;
    }
    return nullptr;
}

const Node* Node::child(const std::string& child_name) const noexcept {
    for (const auto& c : children) {
        if (c->name == child_name) return c.get();
    }
    return nullptr;
}

std::vector<const Node*> Node::childrenNamed(const std::string& child_name) const {
    std::vector<const Node*> out;
    for (const auto& c : children) {
        if (c->name == child_name) out.push_back(c.get());
    }
    return out;
}

std::string Node::childText(const std::string& child_name) const {
    const Node* c = child(child_name);
    return c != nullptr ? c->text : std::string();
}

namespace {

class Parser {
public:
    explicit Parser(const std::string& doc) : s_(doc) {}

    Result<std::unique_ptr<Node>> run() {
        skipMisc();
        if (!at('<')) return error("expected root element");
        auto root = element();
        if (!root) return root;
        skipMisc();
        if (pos_ < s_.size()) return error("unexpected content after root element");
        return root;
    }

private:
    bool at(char c) const { return pos_ < s_.size() && s_[pos_] == c; }
    bool startsWith(const char* text) const { return s_.compare(pos_, std::char_traits<char>::length(text), text) == 0; }

    Status error(const std::string& message) const {
        std::size_t line = 1;
        for (std::size_t i = 0; i < pos_ && i < s_.size(); ++i) {
            if (s_[i] == '\n') ++line;
        }
        return {ErrorCode::InvalidConfig, "XML line " + std::to_string(line) + ": " + message};
    }

    void skipWs() {
        while (pos_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[pos_]))) ++pos_;
    }

    bool skipUntil(const char* terminator) {
        const std::size_t found = s_.find(terminator, pos_);
        if (found == std::string::npos) return false;
        pos_ = found + std::char_traits<char>::length(terminator);
        return true;
    }

    // Whitespace, comments, processing instructions and DOCTYPE.
    void skipMisc() {
        for (;;) {
            skipWs();
            if (startsWith("<?")) {
                if (!skipUntil("?>")) pos_ = s_.size();
            } else if (startsWith("<!--")) {
                if (!skipUntil("-->")) pos_ = s_.size();
            } else if (startsWith("<!DOCTYPE")) {
                if (!skipUntil(">")) pos_ = s_.size();
            } else {
                return;
            }
        }
    }

    std::string name() {
        const std::size_t start = pos_;
        while (pos_ < s_.size()) {
            const char c = s_[pos_];
            if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == ':' || c == '-' || c == '.') {
                ++pos_;
            } else {
                break;
            }
        }
        return s_.substr(start, pos_ - start);
    }

    static void appendUtf8(std::string& out, unsigned long cp) {
        if (cp < 0x80) {
            out += static_cast<char>(cp);
        } else if (cp < 0x800) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }

    static std::string decode(const std::string& raw) {
        std::string out;
        out.reserve(raw.size());
        for (std::size_t i = 0; i < raw.size(); ++i) {
            if (raw[i] != '&') {
                out += raw[i];
                continue;
            }
            const std::size_t semi = raw.find(';', i);
            if (semi == std::string::npos) {
                out += raw[i];
                continue;
            }
            const std::string entity = raw.substr(i + 1, semi - i - 1);
            if (entity == "lt") out += '<';
            else if (entity == "gt") out += '>';
            else if (entity == "amp") out += '&';
            else if (entity == "quot") out += '"';
            else if (entity == "apos") out += '\'';
            else if (!entity.empty() && entity[0] == '#') {
                const bool hex = entity.size() > 1 && (entity[1] == 'x' || entity[1] == 'X');
                appendUtf8(out, std::strtoul(entity.c_str() + (hex ? 2 : 1), nullptr, hex ? 16 : 10));
            } else {
                out += raw.substr(i, semi - i + 1);
            }
            i = semi;
        }
        return out;
    }

    Result<std::unique_ptr<Node>> element() {
        ++pos_;  // '<'
        auto node = std::make_unique<Node>();
        node->name = name();
        if (node->name.empty()) return error("invalid element name");

        for (;;) {
            skipWs();
            if (startsWith("/>")) {
                pos_ += 2;
                return node;
            }
            if (at('>')) {
                ++pos_;
                break;
            }
            std::string key = name();
            if (key.empty()) return error("invalid attribute in <" + node->name + ">");
            skipWs();
            if (!at('=')) return error("expected '=' after attribute " + key);
            ++pos_;
            skipWs();
            if (!at('"') && !at('\'')) return error("expected quoted attribute value");
            const char quote = s_[pos_++];
            const std::size_t end = s_.find(quote, pos_);
            if (end == std::string::npos) return error("unterminated attribute value");
            node->attributes.emplace_back(std::move(key), decode(s_.substr(pos_, end - pos_)));
            pos_ = end + 1;
        }

        std::string text;
        for (;;) {
            if (pos_ >= s_.size()) return error("unterminated element <" + node->name + ">");
            if (startsWith("</")) {
                pos_ += 2;
                const std::string closing = name();
                if (closing != node->name) {
                    return error("mismatched closing tag </" + closing + "> for <" + node->name + ">");
                }
                skipWs();
                if (!at('>')) return error("expected '>'");
                ++pos_;
                break;
            }
            if (startsWith("<!--")) {
                if (!skipUntil("-->")) return error("unterminated comment");
                continue;
            }
            if (startsWith("<![CDATA[")) {
                pos_ += 9;
                const std::size_t end = s_.find("]]>", pos_);
                if (end == std::string::npos) return error("unterminated CDATA");
                text += s_.substr(pos_, end - pos_);
                pos_ = end + 3;
                continue;
            }
            if (startsWith("<?")) {
                if (!skipUntil("?>")) return error("unterminated processing instruction");
                continue;
            }
            if (at('<')) {
                auto child = element();
                if (!child) return child;
                node->children.push_back(std::move(child.value()));
                continue;
            }
            const std::size_t next = s_.find('<', pos_);
            const std::size_t end = next == std::string::npos ? s_.size() : next;
            text += decode(s_.substr(pos_, end - pos_));
            pos_ = end;
        }

        // Trim surrounding whitespace of text content.
        const std::size_t first = text.find_first_not_of(" \t\r\n");
        const std::size_t last = text.find_last_not_of(" \t\r\n");
        node->text = first == std::string::npos ? std::string() : text.substr(first, last - first + 1);
        return node;
    }

    const std::string& s_;
    std::size_t pos_ = 0;
};

}  // namespace

Result<std::unique_ptr<Node>> parse(const std::string& document) {
    std::size_t start = 0;
    // Skip UTF-8 BOM.
    if (document.size() >= 3 && static_cast<unsigned char>(document[0]) == 0xEF &&
        static_cast<unsigned char>(document[1]) == 0xBB && static_cast<unsigned char>(document[2]) == 0xBF) {
        start = 3;
    }
    if (start == 0) {
        return Parser(document).run();
    }
    const std::string body = document.substr(start);
    return Parser(body).run();
}

}  // namespace ethercat_motion::xml
