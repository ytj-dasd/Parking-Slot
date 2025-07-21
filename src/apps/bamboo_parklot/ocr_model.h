#pragma once
#include <onnxruntime_cxx_api.h>
#include <cassert>
#include <vector>
#include <string>
#include <memory>

namespace welkin::bamboo {
struct runreturn {
    std::vector<int64_t> shape;
    void* data;
};

enum class USE_DEVICE {
    CUDA,
    CPU,
};

class OCRModel {
public:
    OCRModel(const std::string& model_path, USE_DEVICE device = USE_DEVICE::CUDA);

    std::vector<void*> runRecognize(long long input_lengths, long long x_length, unsigned char* x);
    runreturn runDetect(int64_t height, int64_t width, long long x_length, unsigned char* x);

    ~OCRModel();

private:
    Ort::Env env;
    Ort::SessionOptions session_options;
    Ort::Session* session;
    Ort::AllocatorWithDefaultOptions allocator;
    size_t num_input_nodes;
    size_t num_output_nodes;

    std::vector<std::string> input_node_names_str;
    std::vector<const char*> input_node_names;
    std::vector<std::string> output_node_names_str;
    std::vector<const char*> output_node_names;

    std::vector<int64_t> input_node_dims;
};
}

