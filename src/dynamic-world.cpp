#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <tinyxml2.h>

#include <stdio.h>
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

using std::vector;
using std::string;
using std::sort;
using std::cout;
using std::endl;
using std::ofstream;


struct ModelEvent
{
	ModelEvent() : startTime(-1), endTime(-1), modelName("") {};
	double startTime;
	double endTime;
	string modelName;
};


namespace gazebo
{
	class Factory : public WorldPlugin
	{
		public:
			void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
			{

				this->world = _parent;

				this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Factory::OnUpdate, this, _1));

				string configName = "events/" + this->world->GetName() + "_config.xml";

				Factory::LoadConfig(configName.c_str());
			}

			void OnUpdate(const common::UpdateInfo &)
			{
				static ModelEvent nextStartEvent = Factory::GetNextEvent(this->futureEvents);
				static ModelEvent nextEndEvent = Factory::GetNextEvent(this->currentEvents);

				static double lastUpdate = 0;

				if (world->GetSimTime().Double() > dayDuration)
				{
					world->SetPaused(true);
				}
				
				if (lastUpdate > world->GetSimTime().Double())
				{
					Factory::Reset();
					nextStartEvent = Factory::GetNextEvent(this->futureEvents);
					nextEndEvent = Factory::GetNextEvent(this->currentEvents);
				}

				lastUpdate = world->GetSimTime().Double();



				// start time of next event to start, -1 if no more events
				if (nextStartEvent.startTime != -1 && world->GetSimTime().Double() >= nextStartEvent.startTime)
				{
					cout << "loading model `" << nextStartEvent.modelName << "` at " << world->GetSimTime().Double() << endl;

					this->world->InsertModelFile("model://" + nextStartEvent.modelName);

					Factory::AddEvent(this->currentEvents, nextStartEvent, Factory::SortByEnd);
					Factory::PopEvent(this->futureEvents);

					nextStartEvent = Factory::GetNextEvent(this->futureEvents);
					nextEndEvent = Factory::GetNextEvent(this->currentEvents);
				}

				// start time of next event to end, -1 if no events are in progress
				if (nextEndEvent.startTime != -1 && world->GetSimTime().Double() >= nextEndEvent.endTime)
				{
					cout << "deleting model `" << nextEndEvent.modelName << "` at " << world->GetSimTime().Double() << endl;

					Factory::DeleteModel(nextEndEvent.modelName);
					Factory::PopEvent(this->currentEvents);
					nextEndEvent = Factory::GetNextEvent(this->currentEvents);
				}
			}

			void Reset()
			{
				cout << "resetting world..." << endl;

				for (int i = 0; i < this->currentEvents.size(); i++)
				{
					cout << "\tdeleting model `" << this->currentEvents[i].modelName << "`" << endl;
					Factory::DeleteModel(this->currentEvents[i].modelName);
				}

				this->currentEvents.clear();
				this->futureEvents.clear();

				for (int i = 0; i < this->modelEvents.size(); i++)
				{
					Factory::AddEvent(this->futureEvents, this->modelEvents[i], Factory::SortByStart);
				}
			}

			// loads config from XML
			void LoadConfig(const char *filename)
			{
				// make sure event lists are clear
				this->modelEvents.clear();
				this->futureEvents.clear();
				this->currentEvents.clear();

				tinyxml2::XMLDocument doc;
				doc.LoadFile(filename);

				if (doc.ErrorID() != 0)
				{
					cout << "error when loading config file `" << filename << "`, " << doc.ErrorID() << endl;
					exit(1);
				}

				// fetch root
				tinyxml2::XMLElement *pRoot = doc.FirstChildElement("day");

				if (pRoot == NULL)
				{
					cout << "error loading root element" << endl;
					exit(1);
				}

				// get length of day
				Factory::CheckXML(pRoot->QueryDoubleAttribute("duration", &(this->dayDuration)));

				// loop through events and add to event list
				tinyxml2::XMLElement *pEvent = pRoot->FirstChildElement("event");

				while (pEvent != NULL)
				{
					// check required attributes
					tinyxml2::XMLElement *pStartTime = pEvent->FirstChildElement("start_time");
					tinyxml2::XMLElement *pDuration = pEvent->FirstChildElement("duration");
					tinyxml2::XMLElement *pModelName = pEvent->FirstChildElement("model");
					tinyxml2::XMLElement *pRepeat = pEvent->FirstChildElement("repeat");

					if (pStartTime == NULL || pDuration == NULL || pModelName == NULL)
					{
						cout << "error loading event attributes" << endl;
						exit(1);
					}

					ModelEvent newEvent;
					double duration;

					// load required attributes
					Factory::CheckXML(pStartTime->QueryDoubleAttribute("value", &(newEvent.startTime)));
					Factory::CheckXML(pDuration->QueryDoubleAttribute("value", &duration));
					newEvent.modelName = pModelName->Attribute("name");

					// duration must be positive...
					if (duration <= 0)
					{
						cout << "duration must be positive (>= 0)" << endl;
						exit(1);
					}

					newEvent.endTime = duration + newEvent.startTime;

					double every;
					int times = 1;

					// repeat is optional
					if (pRepeat != NULL)
					{
						Factory::CheckXML(pRepeat->QueryIntAttribute("times", &times));
						Factory::CheckXML(pRepeat->QueryDoubleAttribute("every", &every));
					}

					for (int i = 0; i < times; i++)
					{
						// add to event list
						if (newEvent.endTime > this->dayDuration)
						{
							cout << "event `" << newEvent.modelName << "` occurs after the end of day, ignoring..." << endl;
						}
						else
						{
							this->modelEvents.push_back(newEvent);
							Factory::AddEvent(this->futureEvents, newEvent, Factory::SortByStart);
						}

						newEvent.startTime += every;
						newEvent.endTime += every;
					}

					pEvent = pEvent->NextSiblingElement("event");
				}
			}

			// verifies XML was read correctly
			void CheckXML(int errCode)
			{
				if (errCode != tinyxml2::XML_SUCCESS)
				{
					cout << "XML parsing error:" << errCode << endl;
					exit(1);
				}
			}

			// fetches next event, assumes reverse sorted vector
			ModelEvent GetNextEvent(const vector< ModelEvent > &events)
			{
				if (!events.empty())
				{
					return events.back();
				}
				else
				{
					return ModelEvent();
				}
			}

			// add event and maintain sorting
			void AddEvent(vector< ModelEvent > &events, ModelEvent event, bool(*compare)(ModelEvent, ModelEvent))
			{
				events.push_back(event);
				sort(events.begin(), events.end(), *compare);
			}

			// remove event
			void PopEvent(vector< ModelEvent > &events)
			{
				events.pop_back();
			}

			// as implied..
			static bool SortByStart(ModelEvent e1, ModelEvent e2)
			{
				return e1.startTime > e2.startTime;
			}

			// as implied..
			static bool SortByEnd(ModelEvent e1, ModelEvent e2)
			{
				return e1.endTime > e2.endTime;
			}

			// removes model from world
			// http://answers.gazebosim.org/question/5887/how-to-delete-a-model-with-an-world-plugin/
			void DeleteModel(const string &name)
			{
				// Info world_ is of type physics::WorldPtr 
				physics::ModelPtr p = this->world->GetModel(name);

				if (p != NULL)
				{
					p->Fini();
				}
				else 
				{
					cout << "Tried to delete non-existent model `" << name << "`" << endl;
				}
			}

		private:
			physics::WorldPtr world;

			event::ConnectionPtr updateConnection;

			// how long is a 'day'
			double dayDuration;

			// list of all events, used to initialize
			// future events at day start
			vector< ModelEvent > modelEvents;

			// list of events yet to occur
			// sorted by start time in reverse order
			vector< ModelEvent > futureEvents;

			// list of events that are currently taking place
			// sorted by end time in reverse order
			vector< ModelEvent > currentEvents;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_WORLD_PLUGIN(Factory)
}