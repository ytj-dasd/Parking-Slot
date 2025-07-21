#include "ocr_model.h"
#include <iostream>
#include <fstream>

namespace welkin::bamboo {
OCRModel::OCRModel(const std::string& model_path, USE_DEVICE device)
{
    try {
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
        this->env = std::move(env);
    }
    catch (const Ort::Exception& e) {
        std::cerr << "Failed to initialize ONNX Runtime environment: " << e.what() << std::endl;
        throw;
    }

    this->session_options.SetInterOpNumThreads(1);
    this->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    
    switch (device)
    {
        case USE_DEVICE::CUDA:
            try {
                this->session_options.AppendExecutionProvider_CUDA(OrtCUDAProviderOptions{});
                // std::cout << "Using CUDA Execution Provider." << std::endl;
            }
            catch (const Ort::Exception& e) {
                std::cerr << "Failed to append CUDA Execution Provider: " << e.what() << std::endl;
                throw;
            }
            break;

        case USE_DEVICE::CPU:
            // std::cout << "Using CPU Execution Provider." << std::endl;
            break;

        default:
            std::cerr << "Unknown device type. Falling back to CPU." << std::endl;
            break;
    }

    std::ifstream model_file(model_path);
    if (!model_file.good()) {
        std::cerr << "模型文件不存在: " << model_path << std::endl;
        exit(EXIT_FAILURE);
    }
    model_file.close();

    try {
        this->session = new Ort::Session(this->env, model_path.c_str(), this->session_options);
    }
    catch (const Ort::Exception& e) {
        std::cerr << "Failed to create ONNX Runtime session: " << e.what() << std::endl;
        throw;
    }

    this->num_input_nodes = this->session->GetInputCount();
    this->input_node_names_str.reserve(this->num_input_nodes);
    this->input_node_names.reserve(this->num_input_nodes);

    // std::cout << "输入节点数量 = " << this->num_input_nodes << std::endl;

    for (size_t i = 0; i < this->num_input_nodes; ++i) {
        Ort::AllocatedStringPtr input_name_ptr = this->session->GetInputNameAllocated(i, allocator);
        this->input_node_names_str.emplace_back(input_name_ptr.get());
        this->input_node_names.emplace_back(this->input_node_names_str.back().c_str());
        // std::cout << "输入 " << i << " : " << this->input_node_names_str.back() << std::endl;
    }

    this->num_output_nodes = this->session->GetOutputCount();
    this->output_node_names_str.reserve(this->num_output_nodes);
    this->output_node_names.reserve(this->num_output_nodes);

    for (size_t i = 0; i < this->num_output_nodes; ++i) {
        Ort::AllocatedStringPtr output_name_ptr = this->session->GetOutputNameAllocated(i, allocator);
        this->output_node_names_str.emplace_back(output_name_ptr.get());
        this->output_node_names.emplace_back(this->output_node_names_str.back().c_str());
        // std::cout << "输出节点名称: " << this->output_node_names_str.back() << std::endl;
    }
}

std::vector<void*> OCRModel::runRecognize(long long input_lengths, long long x_length, unsigned char* x) { 
    size_t input_tensor_size = x_length;
    std::vector<float> input_tensor_values(input_tensor_size);
    for (size_t i = 0; i < input_tensor_size; ++i) {
        input_tensor_values[i] = static_cast<float>(x[i]) / 255.0f;
    }

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<int64_t> input_dims = {1, 1, 32, input_lengths};
    Ort::Value input_tensor1 = Ort::Value::CreateTensor<float>(
        memory_info, 
        input_tensor_values.data(), 
        input_tensor_size, 
        input_dims.data(), 
        input_dims.size()
    );

    int64_t length_value = input_lengths;
    std::vector<int64_t> length_dims = {1};
    Ort::Value input_tensor2 = Ort::Value::CreateTensor<int64_t>(
        memory_info,
        &length_value,
        1,
        length_dims.data(),
        length_dims.size()
    );

    std::vector<Ort::Value> input_tensors;
    input_tensors.emplace_back(std::move(input_tensor1));
    input_tensors.emplace_back(std::move(input_tensor2));

    size_t output_count = this->session->GetOutputCount();
    std::vector<Ort::AllocatedStringPtr> output_name_ptrs;
    std::vector<const char*> output_node_names_local(output_count);
    
    for (size_t i = 0; i < output_count; ++i) {
        output_name_ptrs.emplace_back(this->session->GetOutputNameAllocated(i, allocator));
        output_node_names_local[i] = output_name_ptrs.back().get();
    }

    Ort::RunOptions run_options;
    run_options.SetRunLogVerbosityLevel(4);

    try {
        auto output_tensors = this->session->Run(
            run_options, 
            this->input_node_names.data(), 
            input_tensors.data(), 
            input_tensors.size(), 
            output_node_names_local.data(), 
            output_node_names_local.size()
        );

        int64_t* output_lengths = output_tensors[1].GetTensorMutableData<int64_t>();
        float* logits = output_tensors[0].GetTensorMutableData<float>();

        auto output_info = output_tensors[0].GetTensorTypeAndShapeInfo();
        auto shape = output_info.GetShape();
        // std::cout << "Shape: ";
        // for (auto i : shape){
        //     std::cout << i << " ";
        // }
        // std::cout << std::endl;

        std::vector<void*> ret = { (void*)output_lengths, (void*)logits };
        return ret;
    }
    catch (const Ort::Exception& e) {
        return {};
    }
}

runreturn OCRModel::runDetect(int64_t height, int64_t width, long long x_length, unsigned char* x) {
    size_t input_tensor_size = x_length;
    std::vector<float> input_tensor_values(input_tensor_size);
    size_t output_count = this->session->GetOutputCount();
    std::vector<Ort::AllocatedStringPtr> output_name_ptrs;
    std::vector<const char*> output_node_names_local(output_count);
    
    for (size_t i = 0; i < output_count; ++i) {
        output_name_ptrs.emplace_back(this->session->GetOutputNameAllocated(i, allocator));
        const char* output_name = output_name_ptrs.back().get();
        output_node_names_local[i] = output_name;
    }

    for(int c = 0; c < 3; ++c){
        for(int h_idx = 0; h_idx < height; ++h_idx){
            for(int w_idx = 0; w_idx < width; ++w_idx){
                int hw_idx = h_idx * width + w_idx;
                int img_idx = (h_idx * width + w_idx) * 3 + c; 
                int model_c = (c == 0) ? 2 : (c == 2) ? 0 : 1; 
                float pixel = static_cast<float>(x[img_idx]) / 255.0f;
                input_tensor_values[model_c * height * width + hw_idx] = pixel;
            }
        }
    }

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<int64_t> input_dims = {1, 3, height, width};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, 
        input_tensor_values.data(), 
        x_length, 
        input_dims.data(), 
        input_dims.size());

    std::vector<Ort::Value> input_tensors;
    input_tensors.emplace_back(std::move(input_tensor));

    auto output_tensors = this->session->Run(
        Ort::RunOptions{nullptr}, 
        this->input_node_names.data(), 
        input_tensors.data(), 
        input_tensors.size(), 
        output_node_names_local.data(), 
        output_node_names_local.size()
    );

    float* output_data = output_tensors[0].GetTensorMutableData<float>();

    auto output_info = output_tensors[0].GetTensorTypeAndShapeInfo();
    auto shape = output_info.GetShape();
    // std::cout << "\t形状: ";
    // for(auto dim : shape){
    //     std::cout << "\t" << dim << " ";
    // }

    runreturn ret = {shape, output_data};
    return ret;
}

OCRModel::~OCRModel()
{
    delete this->session;
}
}