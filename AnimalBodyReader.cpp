#include "AnimalBodyReader.h"
#include "..\BBLib\BBWinUtils.h"
#include "..\BBLib\BBUnitsConv.h"
#include <list>
#include <math.h>



AnimalBodyReader::AnimalBodyReader()
{
	strcpy(_body.head.name, "Head");
	strcpy(_body.neck.name, "Neck");
	strcpy(_body.back.name, "Back");
	strcpy(_body.tail.name, "Tail");
	strcpy(_body.legs[LEG_FRONT_LEFT ].name, "LegFL");
	strcpy(_body.legs[LEG_FRONT_RIGHT].name, "LegFR");
	strcpy(_body.legs[LEG_BACK_LEFT  ].name, "LegBL");
	strcpy(_body.legs[LEG_BACK_RIGHT ].name, "LegBR");
	_lastUsedFrameTime = 0;
}

int AnimalBodyReader::StartVideo()
{
	if (_config.readFromLiveStream) {
		if (!_capture.open(_config.deviceId)) {
			return 1;
		}
	}
	else {
		if (!_capture.open(_config.filename)) {
			return 2;
		}
	}

	//set height and width of capture frame
	_frameWidth = (int)_capture.get(CV_CAP_PROP_FRAME_WIDTH);
	_frameHeight = (int)_capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	_fps = (int)_capture.get(CV_CAP_PROP_FPS);
	
	if (_config.readFromLiveStream) {
		printf("Starting live video from device ID=%d\n", _config.deviceId);
		printf("Video frame size: %d x %d\n", _frameWidth, _frameHeight);
	}
	else {
		printf("Starting video from file: %s\n", _config.filename);
		printf("Video frame size: %d x %d, FPS: %d\n", _frameWidth, _frameHeight, _fps);
	}

	// use camera height and HFOV to convert pixels to cm
	double horViewInCm = tan(BB_DEG_2_RAD(_config.HFOV / 2.0)) * _config.height * 2.0;
	_postureAnalyzer.SetPixelsInCm(_frameWidth / horViewInCm);

	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	waitKey(500);
	return 0; // no error
}


int AnimalBodyReader::HandleVideoFrame()
{
	static Mat dst, detected_edges, src, src_gray;
	static int delayMs = 5000 / _fps;
	Mat blurred, threshold, frame, canny_output;
	int min = 0, max = 1000;
	int thresh = 100;
	std::vector<cv::Mat> contours;
	std::vector<cv::Vec4i> hierarchy;

	clock_t now = clock();
	int msSinceLastUsedFrame = int(double(now - _lastUsedFrameTime) / CLOCKS_PER_SEC * 1000.0);
	
	// store image to matrix
	if (!_capture.read(_cameraFeed)) {
		if (_config.readFromLiveStream) {
			printf("Error reading live video stream\n");
			return 1;
		}
		else {
			printf("Finished playing video file\n");
			return 2;
		}
	}
	char mainWindow[] = "Main";
	char countoursOutput[] = "COutput";
	//char thresholdWindow[] = "Threshold";
	//namedWindow(mainWindow, 0);
	//namedWindow(thresholdWindow, 0);

	if (msSinceLastUsedFrame >= _config.frameInterval)
	{
		src = _cameraFeed;
		if (!src.data) return 3;
		_lastUsedFrameTime = clock();

		//convert frame from BGR to HSV colorspace
		cvtColor(_cameraFeed, _HSV, COLOR_BGR2HSV);
		// find all white pixel and set alpha value to zero:
		for (int y = 0; y < _HSV.rows; ++y)
			for (int x = 0; x < _HSV.cols; ++x)
			{
				cv::Vec4b & pixel = _HSV.at<cv::Vec4b>(y, x);
				// if pixel is white
				if (pixel[0] == 255 && pixel[1] == 255 && pixel[2] == 255)
				{
					// set alpha to zero:
					pixel[3] = 0;
				}
			}
		GaussianBlur(_HSV, _HSVGBlur, Size(5,5),0,0);
		medianBlur(_HSVGBlur, _HSVMedianBlur, 21);

		//Search of objects with Canny algorithm
		//Canny(_HSV, canny_output, thresh, thresh * 2, 3);
		//Canny(_HSVGBlur, canny_output, thresh, thresh * 2, 3);
		Canny(_HSVMedianBlur, canny_output, thresh, thresh * 2, 3);

		/* поиск контуров */
		findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		
		/// Get the moments
		vector<Moments> mu(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mu[i] = moments(contours[i], false);
		}

		/// Calculate the area with the moments 00 and compare with the result of the OpenCV function
		printf("\t Info: Area and Contour Length \n");
		for (int i = 0; i < contours.size(); i++)
		{
			printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength(contours[i], true));
			if (mu[i].m00<50)  contours.erase(contours.begin() +i);
		}
	    

		
		
		for (int i = 0; i< contours.size(); i++) {
			cv::Mat iCountur = contours.at(i);
			//cv::Size mSize=iCountur.size();
			//int myArea = mSize.area;
			//printf(iCountur.size());
			double areac = contourArea(iCountur);
			printf("Video frame size: %d", areac);
		}

		/// отрисовка контуров
		Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
		for (int i = 0; i< contours.size(); i++) {
			Scalar color = Scalar(255, 200, 20);
			drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
		}

		/* показ результата в отдельном окне */
		namedWindow(countoursOutput, CV_WINDOW_AUTOSIZE);
		imshow(countoursOutput, drawing);




		////Finding objects by countur
		////medianBlur(_HSV, blurred, 21);
		//for (int y = 0; y < _cameraFeed.rows; y++) {
		//	for (int x = 0; x < _cameraFeed.cols; x++) {
		//		int value = _cameraFeed.at<uchar>(y, x);
		//		if (value == 0) {
		//			Rect rect;
		//			//int count = floodFill(threshold, Point(x, y), Scalar(200), &rect);
		//			//if (rect.width >= min && rect.width <= max
		//			//	&& rect.height >= min && rect.height <= max) {
		//				rectangle(_HSV, rect, Scalar(255, 0, 255, 4));

		//			//}
		//		}
		//	}
		//}
		//imshow(mainWindow, _HSV);












		//imshow(thresholdWindow, threshold);
		//imshow("Output", frame);
		//imshow("Output", threshold);

		// search for stickers
		//for (unsigned int i = 0; i < _stickers.size(); i++) {
		//	if (_stickers[i].minHSV[0] <= _stickers[i].maxHSV[0]) {
		//		inRange(_HSV, _stickers[i].minHSV, _stickers[i].maxHSV, _threshold1);
		//	}
		//	else {
		//		// red -> two ranges
		//		Scalar minHSV, maxHSV;
		//		// lower range
		//		minHSV = _stickers[i].minHSV; minHSV[0] = 0;
		//		maxHSV = _stickers[i].maxHSV;
		//		inRange(_HSV, minHSV, maxHSV, _threshold1);
		//		// upper range
		//		minHSV = _stickers[i].minHSV;
		//		maxHSV = _stickers[i].maxHSV; maxHSV[0] = 179;
		//		inRange(_HSV, minHSV, maxHSV, _threshold2);
		//		// combine the two ranges
		//		_threshold1 |= _threshold2;
		//	}
		//	MorphOps(_threshold1);
		//	FindObjectsForSticker(i);
		//}

		//GetBodyPositionFromStickers();

		//_postureAnalyzer.CalculateBodyPosture(_body);
		//_postureAnalyzer.GetPosture(_posture);
		//char postureText[200];
		//_postureAnalyzer.GetPostureText(_posture, postureText);
		//char timeStr[30];
		//BBUtGetTimeStr(timeStr, 30, ':', TRUE);
		//printf("%s %s\n", timeStr, postureText);

		//DrawBodyLines();
		//for (unsigned int i = 0; i < _stickers.size(); i++) {
		//	DrawSticker(i);
		//}

		imshow("Output", _cameraFeed);
	}

	//delay so that screen can refresh.
	//image will not appear without this waitKey() command
	// 27 is the ASCII code of ESC (exit)
	// 32 is the ASCII code of Space (pause / unpause)
	int key1 = waitKey(delayMs);
	if (key1 == 27) return 12345;
	if (key1 == 32) {
		while (true) {
			int key2 = waitKey(delayMs);
			if (key2 == 27) return 12345;
			if (key2 == 32) break;
		}
	}

	return 0;
}


void AnimalBodyReader::FindObjectsForSticker(int stickerIndex)
{
	// clear sticker's objects from the previously-processed frame
	_stickers[stickerIndex].objects.clear();

	list<Object> objects;
	Mat temp;
	_threshold1.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	if (hierarchy.size() > 0) {
		int numObjects = (int)hierarchy.size();
		//if number of objects greater than maxObjects we assume we have a noisy filter
		if (numObjects < maxObjects) {
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				int area = int(moment.m00);

				//if the area is less than minimum then it is probably just noise
				//if the area is bigger than maximum, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area > _config.minPixels && area < _config.maxPixels * int(_stickers[stickerIndex].size)) 	{

					Object object;
					object.type = string(_stickers[stickerIndex].name);
					object.area = area;
					//printf("%d\n", area);
					object.xPos = int(moment.m10 / area);
					object.yPos = int(moment.m01 / area);
					if (object.xPos < _frameWidth && object.yPos < _frameHeight) {
						object.color = _cameraFeed.at<Vec3b>(Point(object.xPos, object.yPos));
					}
					else {
						// for some reason once in a while it happens...
						object.color = Vec3b(100, 100, 100);
					}
					object.contours.push_back(contours[index]);
					object.hierarchy.push_back(hierarchy[index]);
					objects.push_back(object);
				}
			} 

			// the following code finds up to qty (config) biggest objects for the given sticker,
			// and these objects are added tio the sticker in the sorted order
			while (objects.size() > 0 && _stickers[stickerIndex].objects.size() < _stickers[stickerIndex].qty) {
				int biggestObjArea = 0;
				std::list<Object>::iterator biggestObjIt, it;
				for (it = objects.begin(); it != objects.end(); it++)
				{
					if (it->area > biggestObjArea) {
						biggestObjIt = it;
						biggestObjArea = it->area;
					}
				}
				_stickers[stickerIndex].objects.push_back(*biggestObjIt);
				objects.erase(biggestObjIt);
			}
		}
		else {
			putText(_cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
		}
	}
}

void AnimalBodyReader::DrawObject(Object &obj)
{
	cv::drawContours(_cameraFeed, obj.contours, 0, obj.color, 2, 8, obj.hierarchy);
	cv::circle(_cameraFeed, cv::Point(obj.xPos, obj.yPos), 5, obj.color);
	//cv::putText(_cameraFeed, "(" + IntToString(obj.xPos) + "," + IntToString(obj.yPos) + ")", cv::Point(obj.xPos + 40, obj.yPos + 10), 1, 1, obj.color);
	if (obj.label == "") {
		cv::putText(_cameraFeed, obj.type, cv::Point(obj.xPos + 30, obj.yPos + 5), 1, 1.2, obj.color);
	}
	else {
		cv::putText(_cameraFeed, obj.label, cv::Point(obj.xPos + 30, obj.yPos + 5), 1, 1.2, obj.color, 2);
	}
}


void AnimalBodyReader::DrawSticker(int stickerIndex)
{
	for (unsigned int i = 0; i < _stickers[stickerIndex].objects.size(); i++)
	{
		DrawObject(_stickers[stickerIndex].objects[i]);
	}
}


void AnimalBodyReader::DrawBodyLines()
{
	if (_body.back.IsVisible() && _body.neck.IsVisible()) {
		cv::line(_cameraFeed, 
			Point(_body.neck.pObject->xPos, _body.neck.pObject->yPos),
			Point(_body.back.pObject->xPos, _body.back.pObject->yPos),
			_body.back.pObject->color, 1);
	}
	else if (_body.back.IsVisible() && _body.head.IsVisible()) {
		// if neck is not visible but head is then draw line from back to head
		cv::line(_cameraFeed,
			Point(_body.head.pObject->xPos, _body.head.pObject->yPos),
			Point(_body.back.pObject->xPos, _body.back.pObject->yPos),
			_body.back.pObject->color, 1);
	}
	if (_body.neck.IsVisible() && _body.head.IsVisible()) {
		cv::line(_cameraFeed,
			Point(_body.head.pObject->xPos, _body.head.pObject->yPos),
			Point(_body.neck.pObject->xPos, _body.neck.pObject->yPos),
			_body.head.pObject->color, 1);
	}
	if (_body.tail.IsVisible() && _body.back.IsVisible()) {
		cv::line(_cameraFeed,
			Point(_body.tail.pObject->xPos, _body.tail.pObject->yPos),
			Point(_body.back.pObject->xPos, _body.back.pObject->yPos),
			_body.tail.pObject->color, 1);
	}
}


void AnimalBodyReader::MorphOps(Mat &thresh)
{
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}


void AnimalBodyReader::GetBodyPositionFromStickers()
{
	// for head, neck, back and tail - straightforward as we only have one sticker
	if (_body.head.pSticker->objects.size() > 0) {
		_body.head.pObject = &_body.head.pSticker->objects[0];
		_body.head.pObject->label = string(_body.head.name);
	}
	else {
		_body.head.pObject = NULL;
	}

	if (_body.neck.pSticker->objects.size() > 0) {
		_body.neck.pObject = &_body.neck.pSticker->objects[0];
		_body.neck.pObject->label = string(_body.neck.name);
	}
	else {
		_body.neck.pObject = NULL;
	}

	if (_body.back.pSticker->objects.size() > 0) {
		_body.back.pObject = &_body.back.pSticker->objects[0];
		_body.back.pObject->label = string(_body.back.name);
	}
	else {
		_body.back.pObject = NULL;
	}

	if (_body.tail.pSticker->objects.size() > 0) {
		_body.tail.pObject = &_body.tail.pSticker->objects[0];
		_body.tail.pObject->label = string(_body.tail.name);
	}
	else {
		_body.tail.pObject = NULL;
	}

	// TODO: read leg positions
}


int AnimalBodyReader::ReadConfig(string filename)
{
	int nRc = 0, i, valInt;
	char valStr[50] = "";
	//double valdouble;
	//int valInt;

	// initialize XML parser
	if (_xmlParser.Init() != NO_ERROR) {
		printf(_FUNC_ "ERROR: Failed to initialize XML Parser\n");
		return 1;
	}

	if (_xmlParser.LoadDocFromFile(filename.c_str()) != NO_ERROR) {
		printf(_FUNC_ "ERROR: Failed to load document from file %s\n", filename.c_str());
		return 2;
	}

	// read general parameters
	if (_xmlParser.GetAttributeString("General/@input", valStr) == NO_ERROR) {
		if (!strcmp(valStr, "Live") || !strcmp(valStr, "live")) {
			_config.readFromLiveStream = true;
		}
		else if (!strcmp(valStr, "File") || !strcmp(valStr, "file")) {
			_config.readFromLiveStream = false;
		}
		else {
			printf(_FUNC_ "Error: Could not determine input type, should be Live or File\n");
			return 3;
		}
	}
	else {
		return 3;
	}
	
	if (_config.readFromLiveStream) {
		if (_xmlParser.GetAttributeInt("General/Live/@deviceId", _config.deviceId) != NO_ERROR) {
			return 3;
		}
	}
	else {
		if (_xmlParser.GetAttributeString("General/File/@name", _config.filename) != NO_ERROR) {
			return 3;
		}
	}
	if (_xmlParser.GetAttributeInt("General/Camera/@height", _config.height) != NO_ERROR || _config.height < 100)
	{
		return 3;
	}
	if (_xmlParser.GetAttributeDouble("General/Camera/@HFOV", _config.HFOV) != NO_ERROR || _config.HFOV < 1.0)
	{
		return 3;
	}
	if (_xmlParser.GetAttributeInt("General/Object/@minPixels", _config.minPixels) != NO_ERROR || _config.minPixels < 1)
	{
		return 3;
	}
	if (_xmlParser.GetAttributeInt("General/Object/@maxPixels", _config.maxPixels) != NO_ERROR ||  _config.minPixels < 1)
	{
		return 3;
	}
	if (_xmlParser.GetAttributeInt("General/Algorithm/@frameInterval", _config.frameInterval) != NO_ERROR || _config.frameInterval < 1)
	{
		return 3;
	}

	// read stickers
	char stickerElem[50];
	for (i = 0; ; i++) {
		sprintf(stickerElem, "Stickers/Sticker%d/@name", i + 1);
		if (_xmlParser.GetAttributeString(stickerElem, valStr, false) == NO_ERROR) {
			_stickers.push_back(Sticker());
			strcpy(_stickers[i].name, valStr);
		}
		else {
			break;
		}
		sprintf(stickerElem, "Stickers/Sticker%d/@minH", i + 1);
		if (_xmlParser.GetAttributeInt(stickerElem, valInt) == NO_ERROR && valInt >= 0 && valInt < 256) {
			_stickers[i].minHSV[0] = (double)valInt;
		}
		else {
			return 4;
		}
		sprintf(stickerElem, "Stickers/Sticker%d/@maxH", i + 1);
		if (_xmlParser.GetAttributeInt(stickerElem, valInt) == NO_ERROR && valInt >= 0 && valInt < 256) {
			_stickers[i].maxHSV[0] = (double)valInt;
		}
		else {
			return 4;
		}
		sprintf(stickerElem, "Stickers/Sticker%d/@minS", i + 1);
		if (_xmlParser.GetAttributeInt(stickerElem, valInt) == NO_ERROR && valInt >= 0 && valInt < 256) {
			_stickers[i].minHSV[1] = (double)valInt;
		}
		else {
			return 4;
		}
		sprintf(stickerElem, "Stickers/Sticker%d/@maxS", i + 1);
		if (_xmlParser.GetAttributeInt(stickerElem, valInt) == NO_ERROR && valInt >= 0 && valInt < 256) {
			_stickers[i].maxHSV[1] = (double)valInt;
		}
		else {
			return 4;
		}
		sprintf(stickerElem, "Stickers/Sticker%d/@minV", i + 1);
		if (_xmlParser.GetAttributeInt(stickerElem, valInt) == NO_ERROR && valInt >= 0 && valInt < 256) {
			_stickers[i].minHSV[2] = (double)valInt;
		}
		else {
			return 4;
		}
		sprintf(stickerElem, "Stickers/Sticker%d/@maxV", i + 1);
		if (_xmlParser.GetAttributeInt(stickerElem, valInt) == NO_ERROR && valInt >= 0 && valInt < 256) {
			_stickers[i].maxHSV[2] = (double)valInt;
		}
		else {
			return 4;
		}
		sprintf(stickerElem, "Stickers/Sticker%d/@size", i + 1);
		if (_xmlParser.GetAttributeInt(stickerElem, valInt) == NO_ERROR && valInt > 0) {
			_stickers[i].size = valInt;
		}
		else {
			return 4;
		}
		sprintf(stickerElem, "Stickers/Sticker%d/@qty", i + 1);
		if (_xmlParser.GetAttributeInt(stickerElem, valInt) == NO_ERROR && valInt >= 0) {
			_stickers[i].qty = valInt;
		}
		else {
			return 4;
		}
	}

	// read body parts and their respective stickers
	if (_xmlParser.GetAttributeString("BodyParts/Head/@sticker", valStr) != NO_ERROR ||
		!(_body.head.pSticker = GetStickerPtrByName(valStr)))  {
		printf(_FUNC_ "Sticker for head is not properly defined!\n");
		return 5;
	}
	if (_xmlParser.GetAttributeString("BodyParts/Neck/@sticker", valStr) != NO_ERROR ||
		!(_body.neck.pSticker = GetStickerPtrByName(valStr))) {
		printf(_FUNC_ "Sticker for neck is not properly defined!\n");
		return 5;
	}
	if (_xmlParser.GetAttributeString("BodyParts/Back/@sticker", valStr) != NO_ERROR ||
		!(_body.back.pSticker = GetStickerPtrByName(valStr))) {
		printf(_FUNC_ "Sticker for back is not properly defined!\n");
		return 5;
	}
	if (_xmlParser.GetAttributeString("BodyParts/Tail/@sticker", valStr) != NO_ERROR ||
		!(_body.tail.pSticker = GetStickerPtrByName(valStr))) {
		printf(_FUNC_ "Sticker for tail is not properly defined!\n");
		return 5;
	}
	if (_xmlParser.GetAttributeString("BodyParts/LegFL/@sticker", valStr) != NO_ERROR ||
		!(_body.legs[LEG_FRONT_LEFT].pSticker = GetStickerPtrByName(valStr))) {
		printf(_FUNC_ "Sticker for front left leg is not properly defined!\n");
		return 5;
	}
	if (_xmlParser.GetAttributeString("BodyParts/LegFR/@sticker", valStr) != NO_ERROR ||
		!(_body.legs[LEG_FRONT_RIGHT].pSticker = GetStickerPtrByName(valStr))) {
		printf(_FUNC_ "Sticker for front right leg is not properly defined!\n");
		return 5;
	}
	if (_xmlParser.GetAttributeString("BodyParts/LegBL/@sticker", valStr) != NO_ERROR ||
		!(_body.legs[LEG_BACK_LEFT].pSticker = GetStickerPtrByName(valStr))) {
		printf(_FUNC_ "Sticker for back left leg is not properly defined!\n");
		return 5;
	}
	if (_xmlParser.GetAttributeString("BodyParts/LegBR/@sticker", valStr) != NO_ERROR ||
		!(_body.legs[LEG_BACK_RIGHT].pSticker = GetStickerPtrByName(valStr))) {
		printf(_FUNC_ "Sticker for back right leg is not properly defined!\n");
		return 5;
	}

	return NO_ERROR;
}


Sticker *AnimalBodyReader::GetStickerPtrByName(char *name)
{
	for (unsigned int i = 0; i < _stickers.size(); i++)
	{
		if (!strcmp(name, _stickers[i].name)) {
			return &_stickers[i];
		}
	}
	return NULL;
}


string AnimalBodyReader::IntToString(int num)
{
	std::stringstream s;
	s << num;
	return s.str();
}