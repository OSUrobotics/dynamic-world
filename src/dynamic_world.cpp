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

#include <cstdlib>
#include <cstdio>

#include <ros/package.h>

using std::vector;
using std::string;
using std::sort;
using std::cout;
using std::endl;
using std::ofstream;


#define INF_REPEAT -1

struct Model
{
	string uri;
	string name;
	double pose[6];
};

struct ModelEvent
{
	ModelEvent() : startTime(-1), endTime(-1) {};
	double startTime;
	double endTime;
	vector<Model> models;
};


namespace gazebo
{
	class Factory : public WorldPlugin
	{
		public:
			void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
			{
				this->world = _parent;
				this->physicsEng = _parent->GetPhysicsEngine();
				this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Factory::OnUpdate, this, _1));

				string packagePath = ros::package::getPath("dynamic_world");
				string configName = packagePath + "/events/" + this->world->GetName() + "_config.xml";

				Factory::LoadConfig(configName.c_str());

				// make it faster, twice as fast
				this->physicsEng->SetMaxStepSize(0.002);

				// Create a new transport node
				transport::NodePtr newNode(new transport::Node());
				// *(this->node) = *newNode;

				// cout << "init done\n" << endl;
				// Initialize the node with the world name
				newNode->Init(_parent->GetName());
				// cout << "init done\n" << endl;

				// Create a publisher on the ~/factory topic
				this->pub = newNode->Advertise<msgs::Factory>("~/factory");
				// cout << "init done\n" << endl;
			}

			void _OnUpdate(const common::UpdateInfo &)
			{
				static double sim_prev = -1;
				static double real_prev = -1;

				double sim_cur = world->GetSimTime().Double();
				double real_cur = world->GetRealTime().Double();

				// only update once every little while
				if ((sim_cur - sim_prev) > 0.1)
				{
					cout << "sim time ratio: " << (sim_cur - sim_prev) / (real_cur - real_prev) << endl;

					sim_prev = sim_cur;
					real_prev = real_cur;
				}

					
			}

			void OnUpdate(const common::UpdateInfo &)
			{
				static ModelEvent nextStartEvent = Factory::GetNextEvent(this->futureEvents);
				static ModelEvent nextEndEvent = Factory::GetNextEvent(this->currentEvents);

				static double lastUpdate = 0;

				// stop at end of day
				if (world->GetSimTime().Double() > dayDuration * (this->curDay + 1))
				{
					// reset at beginning of day, pause at end of week
					if (++(this->curDay) < this->weekDuration)
					{
						cout << "Good Morning Gazebo!! Starting day " << this->curDay + 1 << endl;
						Factory::NewDay();
						nextStartEvent = Factory::GetNextEvent(this->futureEvents);
						nextEndEvent = Factory::GetNextEvent(this->currentEvents);
					}
					else
					{
						world->SetPaused(true);
					}
				}
				
				// somewhat hacky check for reset
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
					cout << "beginning event" << endl;

					for (int i = 0; i < nextStartEvent.models.size(); i++)
					{
						cout << "loading model `" << nextStartEvent.models[i].name << "` at " << nextStartEvent.startTime << endl;

						// Create the message
						msgs::Factory msg;

						// Model file to load
						msg.set_sdf_filename(nextStartEvent.models[i].uri);
						// msg.set_edit_name("bookshelf", nextStartEvent.models[i].name);

						double *pose = nextStartEvent.models[i].pose;

						// Pose to initialize the model to
						msgs::Set(msg.mutable_pose(), math::Pose(math::Vector3(pose[0], pose[1], pose[2]),
						          math::Quaternion(pose[3], pose[4], pose[5])));

						// Send the message
						this->pub->Publish(msg);

						// this->world->InsertModelSDF(newSDF);
						// this->world->InsertModelFile(nextStartEvent.models[i].uri);
					}
					

					Factory::AddEvent(this->currentEvents, nextStartEvent, Factory::SortByEnd);
					Factory::PopEvent(this->futureEvents);

					nextStartEvent = Factory::GetNextEvent(this->futureEvents);
					nextEndEvent = Factory::GetNextEvent(this->currentEvents);
				}

				// start time of next event to end, -1 if no events are in progress
				if (nextEndEvent.startTime != -1 && world->GetSimTime().Double() >= nextEndEvent.endTime)
				{
					for (int i = 0; i < nextEndEvent.models.size(); i++)
					{
						cout << "deleting model `" << nextEndEvent.models[i].name << "` at " << nextEndEvent.endTime << endl;
						Factory::DeleteModel(nextEndEvent.models[i].name);
					}
					
					Factory::PopEvent(this->currentEvents);
					nextEndEvent = Factory::GetNextEvent(this->currentEvents);
				}
			}

			// TODO: figure out why this isn't called when reset is pressed in client
			void Reset()
			{
				cout << "resetting world..." << endl;

				for (int i = 0; i < this->currentEvents.size(); i++)
				{
					for (int j = 0; j < this->currentEvents[i].models.size(); j++)
					{
						cout << "deleting model `" << this->currentEvents[i].models[j].name << "`" << endl;
						Factory::DeleteModel(this->currentEvents[i].models[j].name);
					}
				}

				this->currentEvents.clear();
				this->futureEvents.clear();

				for (int i = 0; i < this->modelEvents.size(); i++)
				{
					Factory::AddEvent(this->futureEvents, this->modelEvents[i], Factory::SortByStart);
				}
			}

			// TODO: make this not just a copy-pasted Reset() call...
			void NewDay()
			{
				cout << "resetting world..." << endl;

				for (int i = 0; i < this->currentEvents.size(); i++)
				{
					for (int j = 0; j < this->currentEvents[i].models.size(); j++)
					{
						cout << "deleting model `" << this->currentEvents[i].models[j].name << "`" << endl;
						Factory::DeleteModel(this->currentEvents[i].models[j].name);
					}
				}

				this->currentEvents.clear();
				this->futureEvents.clear();

				// reset events with times shifted to accomodate new day
				for (int i = 0; i < this->modelEvents.size(); i++)
				{
					ModelEvent nextDayEvent = this->modelEvents[i];
					nextDayEvent.startTime += this->curDay * this->dayDuration;
					nextDayEvent.endTime += this->curDay * this->dayDuration;
					cout << "resetting model, new time " << nextDayEvent.startTime << endl;
					Factory::AddEvent(this->futureEvents, nextDayEvent, Factory::SortByStart);
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
				Factory::CheckXML(pRoot->QueryIntAttribute("num", &(this->weekDuration)));
				this->curDay = 0;

				// loop through events and add to event list
				tinyxml2::XMLElement *pEvent = pRoot->FirstChildElement("event");

				while (pEvent != NULL)
				{
					tinyxml2::XMLElement *pStartTime = pEvent->FirstChildElement("start_time");
					tinyxml2::XMLElement *pDuration = pEvent->FirstChildElement("duration");
					tinyxml2::XMLElement *pModel = pEvent->FirstChildElement("model");
					tinyxml2::XMLElement *pRepeat = pEvent->FirstChildElement("repeat");

					// check required attributes
					if (pStartTime == NULL || pDuration == NULL || pModel == NULL)
					{
						cout << "error loading event attributes" << endl;
						exit(1);
					}

					ModelEvent newEvent;
					double duration;

					// load required attributes
					Factory::CheckXML(pStartTime->QueryDoubleAttribute("value", &(newEvent.startTime)));
					Factory::CheckXML(pDuration->QueryDoubleAttribute("value", &duration));

					// duration must be positive...
					if (duration <= 0)
					{
						cout << "duration must be positive" << endl;
						exit(1);
					}

					newEvent.endTime = duration + newEvent.startTime;

					// load models for event
					while (pModel != NULL)
					{
						tinyxml2::XMLElement *pModelURI = pModel->FirstChildElement("uri");
						tinyxml2::XMLElement *pModelName = pModel->FirstChildElement("name");
						tinyxml2::XMLElement *pModelPose = pModel->FirstChildElement("pose");

						// attributes are required
						if (pModelURI == NULL || pModelName == NULL || pModelPose == NULL)
						{
							cout << "error loading model attributes" << endl;
							exit(1);
						}

						Model newModel;

						// load attributes
						newModel.uri = pModelURI->Attribute("value");
						newModel.name = pModelName->Attribute("value");

						std::stringstream ss;
						ss << pModelPose->Attribute("value");
						ss >> newModel.pose[0] >> newModel.pose[1] >> newModel.pose[2];
						ss >> newModel.pose[3] >> newModel.pose[4] >> newModel.pose[5];

						// hack-iest fix ever...
						// Gazebo appears to have a bug that loads the map model
						// offset by 17 in both x and y
						newModel.pose[0] -= 17;
						newModel.pose[1] -= 17;

						newEvent.models.push_back(newModel);

						pModel = pModel->NextSiblingElement("model");
					}

					// deal with repeated events
					double every;
					int times = 1;

					// repeat is optional
					if (pRepeat != NULL)
					{
						Factory::CheckXML(pRepeat->QueryIntAttribute("times", &times));
						Factory::CheckXML(pRepeat->QueryDoubleAttribute("every", &every));
					}

					// if repeat set to INF_REPEAT, repeat until end of day
					for (int i = 0; i < times || times == INF_REPEAT; i++)
					{
						// add to event list
						if (newEvent.endTime > this->dayDuration)
						{
							cout << "event occurs after the end of day, ignoring..." << endl;
							break;
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
					cout << "XML parsing error: " << errCode << endl;
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
			physics::PhysicsEnginePtr physicsEng;

			event::ConnectionPtr updateConnection;

			transport::NodePtr node;
			transport::PublisherPtr pub;

			// how long is a day, in seconds
			double dayDuration;
			// how long is a week, in days
			int weekDuration;
			int curDay;

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