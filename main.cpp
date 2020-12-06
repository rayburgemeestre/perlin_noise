// Copyright 2020 Ray Burgemeestre
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "noise.hpp"

#include <SFML/Graphics.hpp>
#include <cstdlib>
#include <random>
#include "fmt/core.h"

#include "framer.hpp"

#define WINDOW
#define VIDEO
//#define LARGE_VIDEO

int main() {
  std::mt19937 mt;

#ifdef LARGE_VIDEO
  const int window_width = 3840;
  const int window_height = 2160;
  const int font_size = 32;
#else
  const int window_width = 1920 / 2.;
  const int window_height = 1080 / 2.;
  const int font_size = 16;
#endif
  sf::Font font;
  font.loadFromFile("monaco.ttf");
  sf::Text text, text2, text3;
  const auto init_text = [&](sf::Text &text, int column) {
    text.setFont(font);
    text.setCharacterSize(font_size);
    text.setFillColor(sf::Color::Red);
    text.setStyle(sf::Text::Bold);
    text.setPosition(5 + window_width / 3 * column, 0);
  };
  init_text(text, 0);
  init_text(text2, 1);
  init_text(text3, 2);

#ifdef WINDOW
  const int bpp = 32;
  sf::RenderWindow window(sf::VideoMode(window_width, window_height, bpp), "perlin");
  window.setVerticalSyncEnabled(true);
#endif

#ifdef VIDEO
  frame_streamer fs("test.h264", frame_streamer::stream_mode::FILE);
  size_t bitrate = (500 * 1024 * 8);
  double fps = 25;
  fs.initialize(bitrate, window_width, window_height, fps);
#endif

  SimplexNoise noise;

  int octaves1 = 6;
  int octaves2 = 6;
  int octaves3 = 6;
  double persistence1 = 0.5;
  double persistence2 = 0.5;
  double persistence3 = 0.5;
  double percentage1 = 1;
  double percentage2 = 1;
  double percentage3 = 1;
  double scale1 = 1;
  double scale2 = 1;
  double scale3 = 1;

  size_t frames = 0;
  size_t max_frames = 300;
  double z = 0;  // time
#ifdef WINDOW
  while (window.isOpen() && frames < max_frames) {
#else
  while (frames < max_frames) {
#endif
    std::cout << "frame: " << frames << std::endl;
#ifdef WINDOW
    sf::Event event;
    while (window.pollEvent(event)) {
      if ((event.type == sf::Event::Closed) ||
          ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Escape))) {
        window.close();
        break;
      }
      if (event.type == sf::Event::Resized) {
        window.setView(sf::View(sf::FloatRect(0, 0, event.size.width, event.size.height)));
      }
    }
    sf::Vector2u windowSize = window.getSize();
    if (!(windowSize.x >= window_width && windowSize.y >= window_height)) {
      continue;
    }
#endif

    sf::Texture texture;
    texture.create(window_width, window_height);
    sf::Uint8 *pixels2;
    pixels2 = (sf::Uint8 *)malloc(window_width * window_height * 4);  // [window_width * window_height * 4] = {0x00};
    size_t index2 = 0;
    z += 1 / 300.;

#ifdef LARGE_VIDEO
    octaves1 = (frames / (double)max_frames) * 12.;
    octaves2 = (frames / (double)max_frames) * 12.;
    octaves3 = (frames / (double)max_frames) * 12.;
    persistence1 = (frames / (double)max_frames);
    persistence2 = (frames / (double)max_frames);
    persistence3 = (frames / (double)max_frames);
    percentage1 = (frames / (double)max_frames);
    percentage2 = (frames / (double)max_frames);
    percentage3 = (frames / (double)max_frames);
    scale1 = (frames / (double)max_frames) * 20;
    scale2 = (frames / (double)max_frames) * 20;
    scale3 = ((frames / (double)max_frames) * 100) + 100;
    max_frames = 60 * 25;
#else
    // used for .gif in README
    octaves1 = 7.;
    octaves2 = 3.;
    octaves3 = 4.;
    persistence1 = .45;
    persistence2 = 0.5;
    persistence3 = 0.5;
    percentage1 = .6;
    percentage2 = .8;
    percentage3 = .8;
    scale1 = 14;
    scale2 = 75;
    scale3 = 50;

    // some more settings
    octaves1 = 1 + (frames/(double)max_frames) * 8.;
    octaves2 = 1 + (frames/(double)max_frames) * 8.;
    octaves3 = 1 + (frames/(double)max_frames) * 10.;
    persistence1 = (frames/(double)max_frames);
    persistence2 = (frames/(double)max_frames);
    persistence3 = (frames/(double)max_frames);
    percentage1 = (1.0 - (frames/(double)max_frames));
    percentage2 = (1.0 - (frames/(double)max_frames));
    percentage3 = (1.0 - (frames/(double)max_frames));
    scale1 = (frames/(double)max_frames) * 20;
    scale2 = (frames/(double)max_frames) * 20;
    scale3 = ((frames/(double)max_frames) * 100) + 100;
#endif

    std::string s = fmt::format("PERLIN NOISE \noctaves: {}\npersistence: {:.2f}\npercentage: {:.2f}\nscale: {:.2f}",
                                octaves1,
                                persistence1,
                                percentage1,
                                scale1);
    std::string s2 = fmt::format("FRACTAL NOISE \noctaves: {}\npersistence: {:.2f}\npercentage: {:.2f}\nscale: {:.2f}",
                                 octaves2,
                                 persistence2,
                                 percentage2,
                                 scale2);
    std::string s3 =
        fmt::format("TURBULENCE NOISE \noctaves: {}\npersistence: {:.2f}\npercentage: {:.2f}\nscale: {:.2f}",
                    octaves3,
                    persistence3,
                    percentage3,
                    scale3);

    text.setString(s);
    text2.setString(s2);
    text3.setString(s3);

    for (unsigned int y = 0; y < (unsigned int)window_height; y++) {
      for (unsigned int x = 0; x < (unsigned int)window_width; x++) {
        double v = 0;
        if (x > ((window_width / 3) * 2))
          v = noise.simplexNoise(SimplexNoise::NoiseTypeEnum::TURBULENCE,
                                 window_width,
                                 octaves3,
                                 persistence3,
                                 percentage3,
                                 scale3,
                                 x,
                                 y,
                                 z);
        else if (x > ((window_width / 3) * 1))
          v = noise.simplexNoise(SimplexNoise::NoiseTypeEnum::FRACTALNOISE,
                                 window_width,
                                 octaves2,
                                 persistence2,
                                 percentage2,
                                 scale2,
                                 x,
                                 y,
                                 z);
        else
          v = noise.simplexNoise(SimplexNoise::NoiseTypeEnum::PERLINNOISE,
                                 window_width,
                                 octaves1,
                                 persistence1,
                                 percentage1,
                                 scale1,
                                 x,
                                 y,
                                 z);
        if (std::isnan(v)) {
          v = 0;
        }
        double r = mt() / (double)mt.max();
        // v = std::clamp((r * 0.2 + 0.8) * v, 0., 1.);

        // RGBA
        pixels2[index2++] = v * 255 + ((1.0 - v) * 42);
        pixels2[index2++] = v * 255 + ((1.0 - v) * 77);
        pixels2[index2++] = v * 255 + ((1.0 - v) * 130);
        pixels2[index2++] = 255;
      }
    }
    texture.update(pixels2, window_width, window_height, 0, 0);
    sf::Sprite sprite(texture);
    sf::RenderTexture rt;
    rt.create(window_width, window_height);
    rt.draw(sprite);
    rt.draw(text);
    rt.draw(text2);
    rt.draw(text3);

#ifdef WINDOW
    window.draw(sprite);
    window.draw(text);
    window.draw(text2);
    window.draw(text3);
    window.display();
#endif
    frames++;

    static std::vector<uint32_t> pixels;
    unsigned char *rawpixels = nullptr;
    auto copy_pixels = [&]() {
      for (unsigned int y = 0; y < (unsigned int)window_height; y++) {
        for (unsigned int x = 0; x < (unsigned int)window_width; x++) {
          pixels[((window_height - 1 - y) * window_width) + x] = *((uint32_t *)rawpixels);
          rawpixels += sizeof(uint32_t) / sizeof(unsigned char);
        }
      }
    };
    pixels.reserve(window_width * window_height);
    static sf::Image img;
    img = rt.getTexture().copyToImage();
    rawpixels = (unsigned char *)img.getPixelsPtr();
    copy_pixels();
#ifdef VIDEO
    fs.add_frame(pixels);
    if (frames == max_frames) {
      for (int i = 0; i < 50; i++) {
        fs.add_frame(pixels);
      }
    }
#endif
    free(pixels2);
  }
#ifdef VIDEO
  fs.finalize();
#endif

  return EXIT_SUCCESS;
}
