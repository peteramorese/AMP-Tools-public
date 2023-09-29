#pragma once

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>

#include "tools/Logging.h"

namespace ampprivate {
class Encrypter;
}

namespace amp {

class Serializer {
    public:
        Serializer(const std::string& filepath, std::ios_base::openmode open_mode = std::ios::out) 
            : m_filepath(filepath) 
            , m_open_mode(open_mode)
        {
            m_emitter << YAML::BeginMap;
        }
        YAML::Emitter& get() {return m_emitter;}
        void done() {
            m_emitter << YAML::EndMap;
            m_emitter << YAML::Newline;
            
            // Make sure the parent path exists so that the file is created
            std::filesystem::path fs_fp(m_filepath);
            if (!std::filesystem::exists(fs_fp.parent_path())) {
                if (!std::filesystem::create_directories(fs_fp.parent_path())) {
                    ERROR("Could not create directory: " << fs_fp.parent_path());
                }
            }
            std::ofstream fout(m_filepath, m_open_mode);
            fout << m_emitter.c_str();
            fout.close();
        }
    private:
        std::ios_base::openmode m_open_mode;
        std::string m_filepath;
        YAML::Emitter m_emitter;
        friend class ampprivate::Encrypter;
};

class Deserializer {
    public:
        Deserializer() : m_valid(false) {}
        Deserializer(const std::string& filepath) : m_valid(true) {
            try {
                m_node = YAML::LoadFile(filepath);
            } catch (YAML::ParserException e) {
                ERROR("Failed to load file" << filepath << " ("<< e.what() <<")");
            }
        }
        Deserializer(const YAML::Node& node) : m_valid(true), m_node(node) {}
        const YAML::Node& get() const {return m_node;}

        operator bool() const {return m_valid;}
    private:
        bool m_valid = false;
        YAML::Node m_node;
};

}