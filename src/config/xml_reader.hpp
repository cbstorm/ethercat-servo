// SPDX-License-Identifier: MIT
//
// Minimal non-validating XML DOM reader sufficient for ESI files:
// elements, attributes, text, CDATA, comments, processing instructions and
// the predefined / numeric character entities. No DTD support.
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ethercat_motion/types.hpp"

namespace ethercat_motion::xml {

struct Node {
    std::string name;
    std::vector<std::pair<std::string, std::string>> attributes;
    std::vector<std::unique_ptr<Node>> children;
    std::string text;

    const std::string* attribute(const std::string& key) const noexcept;
    const Node* child(const std::string& child_name) const noexcept;
    std::vector<const Node*> childrenNamed(const std::string& child_name) const;
    /// Text of the named child, or empty.
    std::string childText(const std::string& child_name) const;
};

Result<std::unique_ptr<Node>> parse(const std::string& document);

}  // namespace ethercat_motion::xml
