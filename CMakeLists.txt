cmake_minimum_required(VERSION 3.5)

idf_component_register(
                       SRCS "Adafruit_TFTLCD.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES arduino Adafruit-GFX-Library)

project(TFTLCD-Library)
