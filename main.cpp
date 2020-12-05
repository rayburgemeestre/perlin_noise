#include <SFML/Graphics.hpp>
#include <cstdlib>

#include "framer.hpp"

int main() {
  const int window_width = 400;
  const int window_height = 300;
  const int bpp = 32;
  double fps = 25;

  sf::RenderWindow window(sf::VideoMode(window_width, window_height, bpp), "perlin");
  window.setVerticalSyncEnabled(true);

  frame_streamer fs("test.h264", frame_streamer::stream_mode::FILE);
  size_t bitrate = (500 * 1024 * 8);
  fs.initialize(bitrate, window_width, window_height, fps);

  size_t frames = 0;
  while (window.isOpen() && frames < 250) {
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

    sf::Texture texture;
    texture.create(windowSize.x, windowSize.y);
    sf::Uint8 pixels2[window_width * window_height * 4] = {0x00};
    size_t index2 = 0;
    for (unsigned int y = 0; y < (unsigned int)window_height; y++) {
      for (unsigned int x = 0; x < (unsigned int) window_width; x++) {
        pixels2[index2++] = frames; // r
        pixels2[index2++] = 0;      // g
        pixels2[index2++] = 0;      // b
        pixels2[index2++] = 255;    // a
      }
    }
    texture.update(pixels2, window_width, window_height, 0, 0);

    sf::Sprite sprite(texture);
    window.draw(sprite);
    window.display();
    frames++;

    fs.add_frame(pixels2);
  }
  fs.finalize();

  return EXIT_SUCCESS;
}
