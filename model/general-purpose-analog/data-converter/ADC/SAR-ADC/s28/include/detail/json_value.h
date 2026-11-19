#pragma once
// Small strict JSON reader implemented with the C++17 standard library.
// No third-party JSON runtime is needed by the model or the test platform.
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace s28_json {
struct Value {
    enum Kind { Null, Number, Boolean, String, Object, Array } kind = Null;
    double number = 0;
    bool boolean = false;
    std::string string;
    std::map<std::string, Value> object;
    std::vector<Value> array;
    const Value& at(const std::string& key) const {
        if (kind != Object || !object.count(key)) throw std::runtime_error("Missing JSON field: " + key);
        return object.at(key);
    }
    bool has(const std::string& key) const { return kind == Object && object.count(key); }
    double as_number() const {
        if (kind != Number || !std::isfinite(number)) throw std::runtime_error("Expected finite JSON number");
        return number;
    }
    std::string as_string() const {
        if (kind != String) throw std::runtime_error("Expected JSON string");
        return string;
    }
    bool as_bool() const {
        if (kind != Boolean) throw std::runtime_error("Expected JSON boolean");
        return boolean;
    }
};
class Reader {
    std::string text;
    std::size_t pos = 0;
    void ws() { while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) ++pos; }
    char take() { if (pos >= text.size()) throw std::runtime_error("Truncated JSON"); return text[pos++]; }
    void expect(char c) { ws(); if (take() != c) throw std::runtime_error("Malformed JSON punctuation"); }
    std::string str() {
        expect('"'); std::string out;
        while (true) {
            char c = take();
            if (c == '"') return out;
            if (static_cast<unsigned char>(c) < 32) throw std::runtime_error("Control character in JSON string");
            if (c == '\\') {
                c = take();
                if (c == 'n') out += '\n'; else if (c == 'r') out += '\r';
                else if (c == 't') out += '\t'; else if (c == 'b') out += '\b';
                else if (c == 'f') out += '\f';
                else if (c == '"' || c == '\\' || c == '/') out += c;
                else throw std::runtime_error("Use literal UTF-8 instead of JSON unicode escapes");
            } else out += c;
        }
    }
    Value read(unsigned depth) {
        if (depth > 64) throw std::runtime_error("JSON nesting limit");
        ws(); if (pos >= text.size()) throw std::runtime_error("Empty JSON value");
        Value v; char c = text[pos];
        if (c == '{') {
            ++pos; v.kind = Value::Object; ws();
            if (pos < text.size() && text[pos] == '}') { ++pos; return v; }
            while (true) {
                auto key = str(); expect(':');
                if (!v.object.emplace(key, read(depth + 1)).second) throw std::runtime_error("Duplicate JSON key");
                ws(); c = take(); if (c == '}') return v; if (c != ',') throw std::runtime_error("Expected comma");
            }
        }
        if (c == '[') {
            ++pos; v.kind = Value::Array; ws();
            if (pos < text.size() && text[pos] == ']') { ++pos; return v; }
            while (true) {
                v.array.push_back(read(depth + 1)); ws(); c = take();
                if (c == ']') return v;
                if (c != ',') throw std::runtime_error("Expected comma");
            }
        }
        if (c == '"') { v.kind = Value::String; v.string = str(); return v; }
        for (const auto& token : {std::string("true"), std::string("false"), std::string("null")}) {
            if (text.compare(pos, token.size(), token) == 0) {
                pos += token.size(); v.kind = token == "null" ? Value::Null : Value::Boolean;
                v.boolean = token == "true"; return v;
            }
        }
        const auto start = pos;
        if (text[pos] == '-') ++pos;
        if (pos >= text.size() || !std::isdigit(static_cast<unsigned char>(text[pos]))) throw std::runtime_error("Bad JSON number");
        if (text[pos] == '0') ++pos;
        else while (pos < text.size() && std::isdigit(static_cast<unsigned char>(text[pos]))) ++pos;
        if (pos < text.size() && text[pos] == '.') {
            ++pos; const auto digits = pos;
            while (pos < text.size() && std::isdigit(static_cast<unsigned char>(text[pos]))) ++pos;
            if (digits == pos) throw std::runtime_error("Bad JSON fraction");
        }
        if (pos < text.size() && (text[pos] == 'e' || text[pos] == 'E')) {
            ++pos; if (pos < text.size() && (text[pos] == '+' || text[pos] == '-')) ++pos;
            const auto digits = pos;
            while (pos < text.size() && std::isdigit(static_cast<unsigned char>(text[pos]))) ++pos;
            if (digits == pos) throw std::runtime_error("Bad JSON exponent");
        }
        v.kind = Value::Number; v.number = std::stod(text.substr(start, pos - start)); v.as_number(); return v;
    }
public:
    explicit Reader(std::string s) : text(std::move(s)) {}
    Value parse() { Value v = read(0); ws(); if (pos != text.size()) throw std::runtime_error("Trailing JSON data"); return v; }
};
inline Value load(const std::string& path) {
    std::ifstream f(path); if (!f) throw std::runtime_error("Cannot read JSON: " + path);
    std::ostringstream s; s << f.rdbuf(); return Reader(s.str()).parse();
}
} // namespace s28_json
