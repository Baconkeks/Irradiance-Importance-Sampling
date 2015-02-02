#include <vector>
#include <mitsuba/render/scene.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <cassert>
#include <cstdio>
#include <stdint.h>
#include <cmath>


MTS_NAMESPACE_BEGIN

class MyEnvMap {

	struct Pixel
	{
		Pixel(float r, float g, float b, float a) :
		r(r), g(g), b(b), a(a)
		{ }

		Pixel(float r, float g, float b) :
			r(r), g(g), b(b), a(1.f)
		{ }

		Pixel() :
			r(0.f), g(0.f), b(0.f), a(0.f)
		{}

		float r, g, b, a; // red, green, blue and alpha pixel value

		bool operator==(Pixel const& p) const {
			return (p.r == r && p.g == g && p.b == b && p.a == a);
		}
	};

	// since we write files in binary format we need to store
	// if the pfm was written on a machine with little endian
	// architecture in order to read it properly on other machines
	inline bool little_endian()
	{
		const int i = 1;
		return (*reinterpret_cast<char const*>(&i) == 1);
	}

	void write_pfm(
		std::string const& f,               // name/path of the file
		std::vector<Pixel> const& image,    // image data
		uint32_t width,                     // width (in pixel) of the image
		uint32_t height,                    // height (in pixel) of the image
		bool flip = true)                   // if filp, image[0] is bottom left, otherwise top left
	{
		assert(image.size() > 0);
		assert(width > 0);
		assert(height > 0);
		assert(image.size() == width*height);

		std::ofstream of(f.c_str(), std::ios::out | std::ios::binary);

		if (!of) {
			std::cout << "Error: Cannot open " << f << " for writing." << std::endl;
			return;
		}

		// write header.
		of << "PF\n" // Color image.
			<< width << " " << height << "\n"
			<< (little_endian() ? "-1" : "1") << "\n"
			<< std::flush;

		// write data
		for (uint32_t Y = 0; Y < height; ++Y) {
			uint32_t y = flip ? height - 1 - Y : Y;
			assert(y >= 0 && y < height);
			for (uint32_t x = 0; x < width; ++x) {
				// 12 bytes = 3 floats per pixel. alpha is not written.
				of.write(reinterpret_cast<char const*>(&image[x + y*width]), 12);
			}
		}

		if (!of) {
			std::cout << "An error occured while writing " << f << std::endl;
		}
	}

	void readCommentsAndEmptyLines(std::ifstream& file)
	{
		std::string line;
		while (true) {
			std::streampos pos = file.tellg();
			getline(file, line);
			if (line.length() > 0 && line.at(0) == '#') {
				std::cout << "comment: " << line << std::endl;
			}
			else {
				file.seekg(pos);
				break;
			}
		}
	}

	void read_pfm(
		std::string const& f,
		std::vector<Pixel>& image,
		uint32_t& width,
		uint32_t& height,
		bool flip = true)
	{
		std::ifstream file(f.c_str(), std::ios::in | std::ios::binary);

		if (!file || !file.is_open()) {
			std::cerr << "Error: Cannot open " << f << " for reading." << std::endl;
			return;
		}

		std::string line;

		// read file type
		getline(file, line);

		readCommentsAndEmptyLines(file);

		// read width, height
		int w, h, endian;
		getline(file, line);
		std::sscanf(line.c_str(), "%i %i", &w, &h);
		assert(w > 0 && h > 0);

		line.clear();
		getline(file, line);
		std::sscanf(line.c_str(), "%i", &endian);

		bool little_endian_read = endian < 0 ? true : false;
		if (little_endian_read != little_endian()) {
			std::cerr << "Error: File was written on machine with different endian encoding." << std::endl;
			return;
		}

		width = uint32_t(w);
		height = uint32_t(h);

		image.resize(width * height);

		readCommentsAndEmptyLines(file);

		// read data
		float p[3];
		for (uint32_t Y = 0; Y < height; ++Y) {
			uint32_t y = flip ? height - 1 - Y : Y;
			assert(y >= 0 && y < height);
			for (uint32_t x = 0; x < width; ++x) {
				// 12 bytes = 3 floats per pixel
				file.read(reinterpret_cast<char*>(p), 12);
				image[x + y*width] = Pixel(p[0], p[1], p[2]);
			}
		}

		if (!file) {
			std::cout << "An error occured while reading " << f << std::endl;
		}
		return;
	}

	/*Das war vorher die main, habs umbenannt.*/
	int testwrite()
	{
		// create simple test image and write it to file
		uint32_t width = 640;
		uint32_t height = 320;

		std::vector<Pixel> image(width*height);
		/**for (uint32_t y = 0; y < height; ++y) {
			for (uint32_t x = 0; x < width; ++x) {
			Pixel pixel;
			if (x % 2 == 0 && y % 2 == 0) pixel = Pixel(1.f, 0.f, 0.f); // red
			else if (x % 2 == 0 && y % 2 == 1) pixel = Pixel(0.f, 1.f, 0.f); // green
			else if (x % 2 == 1 && y % 2 == 0) pixel = Pixel(0.f, 0.f, 1.f); // blue
			else                               pixel = Pixel(0.f, 0.f, 0.f); // black

			image[x + y*width] = pixel;
			}
			}**/

		/*Bild für quadratische Textur mit 1,2,3,4 in der Mitte und 5,6,7,8 außen*/
		/**
			for (uint32_t y = 0; y < height/2; y++){
			for (uint32_t x = 0; x < width / 2; x++){
			Pixel pixel;
			pixel = Pixel(0.7f, 0.3f, 0.7f);//hässliches Lila: unten links / 7,3
			image[x + y*width] = pixel;
			}
			for (uint32_t x = width / 2; x < width; x++){
			Pixel pixel;
			pixel = Pixel(0.f, 1.f, 1.f); //Helltürkis: unten rechts / 4,8
			image[x + y*width] = pixel;
			}
			}
			for (uint32_t y = height/2; y < height; y++){
			for (uint32_t x = 0; x < width / 2; x++){
			Pixel pixel;
			pixel = Pixel(0.f, 0.f, 1.f); //blau: oben links / 2,6
			image[x + y*width] = pixel;
			}
			for (uint32_t x = width / 2; x < width; x++){
			Pixel pixel;
			pixel = Pixel(0.2f, 0.2f, 0.2f); //grau: oben rechts / 1,5
			image[x + y*width] = pixel;
			}
			}**/

		/*Bild für rechteckige Textur mit 1,2,3,4 links und 5,6,7,8 rechts*/
		uint32_t width_half = 320;
		uint32_t height_half = 160;
		uint32_t width_quarter = 160;
		for (int x = 0; x < width_half; x++){
			if (x < width_quarter){
				for (int y = 0; y < x; y++){
					/*2*/
					Pixel pixel;
					pixel = Pixel(0.8f, 0.4f, 0.1f);
					image[x + y*width] = pixel;
				}
				for (int y = x; y < width_half - x; y++){
					/*1*/
					Pixel pixel;
					pixel = Pixel(0.2f, 0.9f, 0.2f);
					image[x + y*width] = pixel;
				}
				for (int y = width_half - x; y < height; y++){
					/*3*/
					Pixel pixel;
					pixel = Pixel(0.2f, 0.2f, 0.9f);
					image[x + y*width] = pixel;
				}
			}
			else {
				for (int y = 0; y < width_half - x; y++){
					/*2*/
					Pixel pixel;
					pixel = Pixel(0.8f, 0.4f, 0.1f);
					image[x + y*width] = pixel;
				}
				for (int y = width_half - x; y < x; y++){
					/*4*/
					Pixel pixel;
					pixel = Pixel(0.6f, 0.6f, 0.6f);
					image[x + y*width] = pixel;
				}
				for (int y = x; y < height; y++){
					/*3*/
					Pixel pixel;
					pixel = Pixel(0.2f, 0.2f, 0.9f);
					image[x + y*width] = pixel;
				}
			}
		}
		for (int x = 0; x < width_half; x++){
			if (x < width_quarter){
				for (int y = 0; y < x; y++){
					/*8*/
					Pixel pixel;
					pixel = Pixel(0.3f, 0.1f, 0.1f);
					image[x + width_half + y*width] = pixel;
				}
				for (int y = x; y < width_half - x; y++){
					/*5*/
					Pixel pixel;
					pixel = Pixel(0.1f, 0.3f, 0.1f);
					image[x + width_half + y*width] = pixel;
				}
				for (int y = width_half - x; y < height; y++){
					/*7*/
					Pixel pixel;
					pixel = Pixel(0.1f, 0.1f, 0.4f);
					image[x + width_half + y*width] = pixel;
				}
			}
			else {
				for (int y = 0; y < width_half - x; y++){
					/*8*/
					Pixel pixel;
					pixel = Pixel(0.4f, 0.1f, 0.1f);
					image[x + width_half + y*width] = pixel;
				}
				for (int y = width_half - x; y < x; y++){
					/*6*/
					Pixel pixel;
					pixel = Pixel(0.3f, 0.3f, 0.3f);
					image[x + width_half + y*width] = pixel;
				}
				for (int y = x; y < height; y++){
					/*7*/
					Pixel pixel;
					pixel = Pixel(0.1f, 0.1f, 0.4f);
					image[x + width_half + y*width] = pixel;
				}
			}
		}


		std::string file("neuertesttest2x1.pfm");
		write_pfm(file, image, width, height);

		// read file again and check if we get the same data
		uint32_t width_read;
		uint32_t height_read;
		std::vector<Pixel> image_read;
		read_pfm(file, image_read, width_read, height_read);

		assert(width = width_read);
		assert(height = height_read);
		for (uint32_t y = 0; y < height; ++y) {
			for (uint32_t x = 0; x < width; ++x) {
				assert(image_read[x + y*width] == image[x + y*width]);
			}
		}

		return 0;
	}


public:
	//Achtung: Koordinatensystem von Kamera ist standardmäßig rechtshändig, paper nutzt linkshändig
	//in toWorld: <scale x="-1"/>
	MyEnvMap::MyEnvMap(){
		////testwrite();
		//std::string file("octamap.pfm"); //muss in mitsuba/dist liegen
		//uint32_t width_read;
		//uint32_t height_read;
		//std::vector<Pixel> image_read;

		//read_pfm(file, image_read, width_read, height_read);

		//width = width_read;
		//height = height_read;
		//image = image_read;
		//use_quadratic_texture = true;
		//debug_farben_direkt_von_texturkoordinaten_abhaengig = false;
	}

	void MyEnvMap::writeImage(std::vector<float> map, uint32_t w, uint32_t h, std::string filename){
		std::vector<Pixel> pixels(w*h);
		for (int i = 0; i < map.size(); i++){
			pixels[i] = Pixel(map[i], map[i], map[i]);
		}

		std::string file(filename);
		write_pfm(file, pixels, w, h);
		width = w;
		height = h;
		image = pixels;
		use_quadratic_texture = true;
		debug_farben_direkt_von_texturkoordinaten_abhaengig = false;
	}

	MyEnvMap::MyEnvMap(std::vector<float> map, uint32_t w, uint32_t h){
		std::vector<Pixel> pixels(w*h);
		for (int i = 0; i < map.size(); i++){
			pixels[i] = Pixel(map[i], map[i], map[i]);
		}

		std::string file("statisticsolidanglemap.pfm");
		write_pfm(file, pixels, w, h);
		width = w;
		height = h;
		image = pixels;
		use_quadratic_texture = true;
		debug_farben_direkt_von_texturkoordinaten_abhaengig = false;
	}

	/*Nimmt Texturkoordinate aus [0,1]^2 und wandelt in nicht normierten Richtungsvektor um*/
	Vector textueToDirection(Point2d p){
		if (use_quadratic_texture){
			return textureQuadToDirection(p);
		}
		else {
			return textureRectToDirection(p);
		}
	}

	/*Das ruf ich in meinem Pathtracer auf, wenn ein Strahl ins Leere geht, anstatt die envmap der Szene zu benutzen*/
	Spectrum getTextureAtDirection(Vector d){
		Point2d p = directionToTextureCoord(d);
		return getTextureAtTextureCoord(p);
	}

	/**
		Wandelt Richtungsvektor in Texturkoordinate \in[0,1]^2 um
		use_quadratic_texture gibt an, ob von quadratischer Textur mit 1,2,3,4 innen ausgegangen wird.
		falls nicht, benutze rechteckige Textur mit 1,2,3,4 links
	**/
	Point2d directionToTextureCoord(Vector d){
		if (d.isZero()) return Point2d(-1, -1);

		if (use_quadratic_texture){ //quadratische Textur mit 1,2,3,4, innen und 5,6,7,8 außen
			return directionToTextureQuadratic(d);
		}
		else { //rechteckige Textur mit 1,2,3,4 links und 5,6,7,8 rechts
			return directionToTextureRectangular(d);
		}
	}

	Spectrum checkIfConversionWorksRect(Vector dir, bool useQuadr){
		Point2d p1, p2;
		Vector dir_abs_norm = dir / (std::abs(dir[0]) + std::abs(dir[1]) + std::abs(dir[2]));
		Vector dir_neu;
		/*Hin-und-zurück-Test (tut beides)*/
		if (useQuadr){
			p1 = directionToTextureQuadratic(dir_abs_norm); //Komisch dass das auch tut, wenn man dir reinwirft
			p2 = directionToTextureQuadratic(textureQuadToDirection(p1));
		}
		else {
			p1 = directionToTextureRectangular(dir_abs_norm);

			p2 = directionToTextureRectangular(textureRectToDirection(p1));
		}
		if (std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y) > 0.0001) return returnGrey();
		return returnGreen();

		/*Hin-zurück-hin-Test (tut beides)*/
		/*if (useQuadr){
			dir_neu = textureQuadToDirection(directionToTextureQuadratic(dir_abs_norm));
		}
		else {
			dir_neu = textureRectToDirection(directionToTextureRectangular(dir_abs_norm));
		}
		if (std::abs(dir_neu[0] - dir_abs_norm[0]) + std::abs(dir_neu[1] - dir_abs_norm[1]) + std::abs(dir_neu[2] - dir_abs_norm[2]) > 0.001){
			return returnGrey();
		}
		return returnGreen();*/
	}


	/*Nimmt Richtung, die schon auf Oktaederoberfläche liegt, und wandelt in Texturkoordinate um*/
	static Point2d directionToTextureQuadratic(Vector d){
		Vector p_ = d / (std::abs(d[0]) + std::abs(d[1]) + std::abs(d[2])); //planare projektion
		Point2d p;
		if (p_[1] >= 0){ //facepalm. hier stand davor p_[2] (also pz). wichtig war aber py
			p.x = p_[0];
			p.y = p_[2];
		}
		else {
			if (p_[0] > 0 && p_[2] > 0){
				p.x = 1 - p_[2];
				p.y = 1 - p_[0];
			}
			else if (p_[0] < 0 && p_[2] > 0){
				p.x = -1 + p_[2];
				p.y = 1 + p_[0];
			}
			else if (p_[0] > 0 && p_[2] < 0){
				p.x = 1 + p_[2];
				p.y = -1 + p_[0];
			}
			else{
				p.x = -1 - p_[2];
				p.y = -1 - p_[0];
			}
		}
		//Skaliere, sodass Ursprung unten links und p\in [0,1]^2
		//TODO Hm, wo ist der Ursprung normalerweise bei der envmap?
		p.x += 1;
		p.y += 1;
		p.x *= 0.5f;
		p.y *= 0.5f;
		return p;
	}

	/*Nimmt Richtung, die nicht unbedingt Oktaederoberfläche liegt...*/
	static Point2d directionToTextureRectangular(Vector d){
		Vector p_ = d / (std::abs(d[0]) + std::abs(d[1]) + std::abs(d[2])); //planare projektion
		Point2d p;
		p.y = p_[0] + p_[2];
		if (p_[1] >= 0){
			p.x = p_[0] - p_[2] - 1;
		}
		else {
			p.x = p_[2] - p_[0] + 1;
		}
		//Skaliere, sodass Ursprung unten links und p\in [0,1]^2
		p.x += 2;
		p.x *= 0.25f;
		p.y += 1;
		p.y *= 0.5f;
		return p;
	}



	/*
		Nimmt Texturkoordinate aus [0,1]^2 für rechteckige Textur und
		wandelt sie in Richtung um (nicht normiert!)
	*/
	static Vector textureRectToDirection(Point2d p_textur){ //TODO gleichmäßig verteilen
		Vector d;
		Point2d p = p_textur;
		p.x = 4 * p.x - 2;
		p.y = 2 * p.y - 1;
		if (p.x <= 0){
			/**	px = dx - dz - 1
				py = dx + dz				2dx = px + py + 1
				dy >= 0
				1 = |dx| + |dy| + |dz|
			**/
			d[0] = 0.5f*(p.x + p.y + 1);
			d[2] = p.y - d[0];
			d[1] = 1 - std::abs(d[0]) - std::abs(d[2]);
		}
		else {
			/**	px = dz - dx + 1
			py = dx + dz				2dx = -px + py + 1
			dy >= 0
			1 = |dx| + |dy| + |dz|
			**/
			d[0] = 0.5f*(-p.x + p.y + 1);
			d[2] = p.y - d[0];
			d[1] = -1 + std::abs(d[0]) + std::abs(d[2]);
		}
		return d;
	}
	private:
	/*
		Nimmt Texturkoordinate aus [0,1]^2 für quadratische Textur und
		wandelt sie in Richtung um (nicht normiert!)
	*/
	Vector textureQuadToDirection(Point2d p_textur){
		Vector d;
		Point2d p = p_textur;
		p.x = p.x * 2 - 1;
		p.y = p.y * 2 - 1;
		if (std::abs(p.x) + std::abs(p.y) <= 1){ //innen -> 1,2,3,4
			d[0] = p.x;
			d[2] = p.y;
			d[1] = 1 - std::abs(p.x) - std::abs(p.y);
		}
		else {
			//nach innen klappen
			if (p.x + p.y > 1){ //oben rechts
				Point2d platzhalter(1 - p.y, 1 - p.x);
				p = platzhalter;
			}
			else if (p.x - p.y > 1){ //unten rechts
				Point2d platzhalter(1 + p.y, -1 + p.x);
				p = platzhalter;
			}
			else if (-p.x + p.y > 1){ //oben links
				Point2d platzhalter(-1 + p.y, 1 + p.x);
				p = platzhalter;
			}
			else { //unten links
				Point2d platzhalter(-1 - p.y, -1 - p.x);
				p = platzhalter;
			}
			d[0] = p.x;
			d[2] = p.y;
			d[1] = -1 + std::abs(p.x) + std::abs(p.y); //geht insgesamt kompakter mit dem von oben, aber unübersichtlich
		}
		return d;
	}

	Spectrum getTextureAtTextureCoord(Point2d p){
		float x = p.x;
		float y = p.y;

		/*
		Check auf welchem achtel x/y liegt, spectrum ausgeben.
		Damit siehts aus als wär man in nem Oktaeder. gut.
		Ich hab mir die Mühe nur für die rechteckige Textur gemacht,
		quadratisch wollte ich dann nicht auch noch machen,
		solange ich eh nicht richtig aus dem Pixelvektor lesen kann*/
	/**	if (debug_farben_direkt_von_texturkoordinaten_abhaengig && !use_quadratic_texture){ 
			x *= 2;
			Spectrum s;
			if (x < 1.f){ //1,2,3,4
				if (y > x){ //1 oder 2
					if (x + y < 1) s.fromLinearRGB(1.0, 0.8, 0.8);//hellrosa //2
					else s.fromLinearRGB(0.8, 1.0, 0.8); //hellgrün //1
				}
				else { //3 oder 4
					if (x + y < 1) s.fromLinearRGB(0.8, 0.8, 1.0);//helllila //3
					else s.fromLinearRGB(1.0, 1.0, 0.7); //hellgelb //4
				}
			}
			else {
				x -= 1;
				if (y > x){ //8 oder 5
					if (x + y < 1) s.fromLinearRGB(0.4, 0.1, 0.1);//dunkelrosa
					else s.fromLinearRGB(0.1, 0.4, 0.1); //dunkelgrün
				}
				else { //7 oder 6
					if (x + y < 1) s.fromLinearRGB(0.1, 0.1, 4.0);//dunkellila
					else s.fromLinearRGB(0.2, 0.2, 0.2); //dunkelgrau
				}
			}
			return s;
		}**/
		
		/*Normales Auslesen aus Pixelvektor. Damit tuts nicht. 
		PixelToSpectrum gibt richtige Farbe, aber envMapPix[x + width*y] gibt falschen Pixel */

		int x_int, y_int;
		if (x == 1) x_int = width - 1; //Damit man am Rand auch noch auslesen kann
		else x_int = floor(x*width);
		if (y == 1) y_int = height - 1;
		else y_int = floor(y*height);
		if (y_int >= height || y_int < 0 || x_int >= width || x_int < 0){ //image nur an gültigen Stellen auslesen
			return returnRed();
		}
		return PixelToSpectrum(image[x_int + width * y_int]);
	}

	Spectrum PixelToSpectrum(Pixel p){

		Spectrum s;
		s.fromLinearRGB(p.r, p.g, p.b); //Pech für den Alpha-Wert. //TODO Hier scheiterts grade. wahrsch gibts noch gar kein file.
		return s;
	}

	Spectrum returnRed(){
		Spectrum s;
		s.fromLinearRGB(1.f, 0.f, 0.f);
		return s;
	}

	Spectrum returnGreen(){
		Spectrum s;
		s.fromLinearRGB(0.f, 1.f, 0.f);
		return s;
	}

	Spectrum returnGrey(){
		Spectrum s;
		s.fromLinearRGB(0.4f, 0.4f, 0.4f);
		return s;
	}

private:
	std::vector<Pixel> image;
	int width;
	int height;
	bool use_quadratic_texture; //true => quadratische Textur, false => rechteckige textur. Im Moment im Konstruktor hart gesetzt, wär vllt hübscher das vom .pfm abhängig zu machen
	bool debug_farben_direkt_von_texturkoordinaten_abhaengig; //liest nicht aus image[...] sondern schaut nur, in welchem Bereich Texturkoordinaten sind

};


MTS_NAMESPACE_END