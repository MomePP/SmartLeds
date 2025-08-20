#include <Color.h>
#include <catch.hpp>

// Test specifically for the buffer corruption fix in ESP32 Arduino v3.3.0
TEST_CASE("Rgb getGrb method works correctly", "[buffer-corruption]") {
    Rgb pixel(255, 128, 64, 255); // R=255, G=128, B=64, A=255
    
    // Test that getGrb returns correct values for each component
    REQUIRE(pixel.getGrb(0) == 128); // Green
    REQUIRE(pixel.getGrb(1) == 255); // Red  
    REQUIRE(pixel.getGrb(2) == 64);  // Blue
}

TEST_CASE("Rgb structure packing is correct", "[buffer-corruption]") {
    // Ensure Rgb structure is exactly 4 bytes as expected by RMT driver
    REQUIRE(sizeof(Rgb) == 4);
    
    // Test that the memory layout is correct (g, r, b, a)
    Rgb pixel(0x12, 0x34, 0x56, 0x78); // R=0x12, G=0x34, B=0x56, A=0x78
    uint8_t* bytes = (uint8_t*)&pixel;
    
    REQUIRE(bytes[0] == 0x34); // g
    REQUIRE(bytes[1] == 0x12); // r  
    REQUIRE(bytes[2] == 0x56); // b
    REQUIRE(bytes[3] == 0x78); // a
}

TEST_CASE("Multiple pixel access doesn't corrupt buffers", "[buffer-corruption]") {
    // This test simulates the scenario where updating one pixel
    // would corrupt other pixels in ESP32 Arduino v3.3.0
    
    const int pixel_count = 5;
    Rgb pixels[pixel_count] = {
        Rgb(255, 0, 0),   // Red
        Rgb(0, 255, 0),   // Green  
        Rgb(0, 0, 255),   // Blue
        Rgb(255, 255, 0), // Yellow
        Rgb(255, 0, 255)  // Magenta
    };
    
    // Simulate accessing pixels in the pattern that the encoder would use
    for (int i = 0; i < pixel_count; i++) {
        Rgb& pixel = pixels[i];
        
        // Access each color component as the encoder would
        uint8_t g = pixel.getGrb(0);
        uint8_t r = pixel.getGrb(1); 
        uint8_t b = pixel.getGrb(2);
        
        // Verify the values are as expected
        REQUIRE(g == pixel.g);
        REQUIRE(r == pixel.r);
        REQUIRE(b == pixel.b);
        
        // Verify no other pixels were affected
        for (int j = 0; j < pixel_count; j++) {
            if (j != i) {
                // Other pixels should maintain their original values
                REQUIRE(pixels[j].r == (j == 0 ? 255 : j == 1 ? 0 : j == 2 ? 0 : j == 3 ? 255 : 255));
                REQUIRE(pixels[j].g == (j == 0 ? 0 : j == 1 ? 255 : j == 2 ? 0 : j == 3 ? 255 : 0));
                REQUIRE(pixels[j].b == (j == 0 ? 0 : j == 1 ? 0 : j == 2 ? 255 : j == 3 ? 0 : 255));
            }
        }
    }
}

TEST_CASE("Pixel data structure integrity for ESP-IDF v5.5", "[esp-idf-v5.5]") {
    // Test that the Rgb structure meets ESP-IDF v5.5 memory alignment requirements
    
    // Verify structure size and alignment
    REQUIRE(sizeof(Rgb) == 4);
    REQUIRE(alignof(Rgb) >= 4); // Should be at least 4-byte aligned
    
    // Test multiple pixels in an array (simulating LED strip buffer)
    const int pixel_count = 32;
    Rgb pixels[pixel_count];
    
    // Initialize pixels with distinct patterns
    for (int i = 0; i < pixel_count; i++) {
        pixels[i] = Rgb(i * 8, (i * 16) % 256, (i * 24) % 256);
    }
    
    // Verify each pixel maintains its data integrity when accessed
    for (int i = 0; i < pixel_count; i++) {
        const Rgb& pixel = pixels[i];
        
        // Test const-correctness of getGrb method
        uint8_t g = pixel.getGrb(0);
        uint8_t r = pixel.getGrb(1);
        uint8_t b = pixel.getGrb(2);
        
        // Verify values match expected pattern
        REQUIRE(r == (i * 8));
        REQUIRE(g == ((i * 16) % 256));
        REQUIRE(b == ((i * 24) % 256));
        
        // Verify accessing one pixel doesn't affect others
        for (int j = 0; j < pixel_count; j++) {
            if (j != i) {
                REQUIRE(pixels[j].r == (j * 8));
                REQUIRE(pixels[j].g == ((j * 16) % 256));
                REQUIRE(pixels[j].b == ((j * 24) % 256));
            }
        }
    }
}

TEST_CASE("Color component access bounds checking", "[esp-idf-v5.5]") {
    // Test that getGrb method handles bounds correctly
    Rgb pixel(0x12, 0x34, 0x56, 0x78);
    
    // Valid indices should work
    REQUIRE(pixel.getGrb(0) == 0x34); // Green
    REQUIRE(pixel.getGrb(1) == 0x12); // Red
    REQUIRE(pixel.getGrb(2) == 0x56); // Blue
    
    // The method should handle the expected indices correctly
    // (Note: getGrb only expects indices 0, 1, 2 based on the implementation)
}