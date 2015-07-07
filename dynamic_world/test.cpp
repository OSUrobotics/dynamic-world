#include <iostream>
#include <tinyxml2.h>

int main()
{
	tinyxml2::XMLDocument doc;
	doc.LoadFile("events/test_event.xml");

	return 0;
}