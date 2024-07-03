#ifndef EXCEPTIONS_HPP
#define EXCEPTIONS_HPP

#include <exception>
#include <string>

class EmergencyException : public std::exception {
public:
    explicit EmergencyException(const std::string& message) : msg_(message) {}
    virtual const char* what() const noexcept override { return msg_.c_str(); }

private:
    std::string msg_;
};

class BadStateException : public std::exception {
public:
    explicit BadStateException(const std::string& message) : msg_(message) {}
    virtual const char* what() const noexcept override { return msg_.c_str(); }

private:
    std::string msg_;
};

#endif