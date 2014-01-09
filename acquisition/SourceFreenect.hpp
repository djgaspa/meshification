#pragma once
#include <memory>
#include "Source.hpp"

class SourceFreenect : public Source
{
    struct Impl;
    std::unique_ptr<Impl> p;
public:
    SourceFreenect(const int id = 0);
    ~SourceFreenect();
    void grab() override;
    void startImage() override;
    void startDepth() override;
    void startIr() override;
    void stopAll() override;
    void getImage(char* rgb) override;
    void getDepth(char* depth) override;
    void getIr(char *ir) override;
    int width() const override;
    int height() const override;
    std::string getSerial() const override;
};
