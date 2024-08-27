#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vosk_api.h>  // VOSK 라이브러리 헤더
#include <alsa/asoundlib.h>
#include <json/json.h>
#include <memory>
#include <string>

class SpeechRecognitionNode : public rclcpp::Node {
public:
    SpeechRecognitionNode() : Node("speech_recognition_node") {
        // ROS2 Publisher 설정
        publisher_ = this->create_publisher<std_msgs::msg::String>("recognized_command", 10);

        // VOSK 모델 로드
        model_ = vosk_model_new("path_to_vosk_model");
        recognizer_ = vosk_recognizer_new(model_, 16000.0);

        // ALSA를 통해 마이크 초기화
        initializeAudio();

        // 타이머로 음성 처리 반복 실행
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SpeechRecognitionNode::processAudio, this)
        );
    }

    ~SpeechRecognitionNode() {
        vosk_recognizer_free(recognizer_);
        vosk_model_free(model_);
        snd_pcm_close(capture_handle_);
    }

private:
    void initializeAudio() {
        int err;
        if ((err = snd_pcm_open(&capture_handle_, "default", SND_PCM_STREAM_CAPTURE, 0)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open audio device (%s)", snd_strerror(err));
            rclcpp::shutdown();
        }

        snd_pcm_hw_params_t *hw_params;
        snd_pcm_hw_params_alloca(&hw_params);
        snd_pcm_hw_params_any(capture_handle_, hw_params);
        snd_pcm_hw_params_set_access(capture_handle_, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(capture_handle_, hw_params, SND_PCM_FORMAT_S16_LE);
        snd_pcm_hw_params_set_rate_near(capture_handle_, hw_params, &rate_, 0);
        snd_pcm_hw_params_set_channels(capture_handle_, hw_params, 1);

        if ((err = snd_pcm_hw_params(capture_handle_, hw_params)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot set parameters (%s)", snd_strerror(err));
            rclcpp::shutdown();
        }

        snd_pcm_prepare(capture_handle_);
    }

    void processAudio() {
        const int buffer_size = 4096;
        int16_t buffer[buffer_size];

        int err = snd_pcm_readi(capture_handle_, buffer, buffer_size / 2);
        if (err != buffer_size / 2) {
            RCLCPP_ERROR(this->get_logger(), "Read from audio interface failed (%s)", snd_strerror(err));
            return;
        }

        if (vosk_recognizer_accept_waveform(recognizer_, (const char*)buffer, sizeof(buffer))) {
            const char* result = vosk_recognizer_result(recognizer_);
            Json::Value root;
            Json::Reader reader;
            reader.parse(result, root);
            std::string text = root["text"].asString();

            if (!text.empty()) {
                RCLCPP_INFO(this->get_logger(), "Recognized Command: %s", text.c_str());

                auto msg = std_msgs::msg::String();
                msg.data = text;
                publisher_->publish(msg);
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // VOSK 관련 변수
    vosk_model_t* model_;
    vosk_recognizer_t* recognizer_;

    // ALSA 관련 변수
    snd_pcm_t* capture_handle_;
    unsigned int rate_ = 16000;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeechRecognitionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}