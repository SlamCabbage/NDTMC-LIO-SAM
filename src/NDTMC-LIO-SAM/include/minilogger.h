//Easy-to-use log classes,called minilogger
#ifndef LOGGER_H
#define LOGGER_H
#include <string>
#include <fstream>

// ============================================================
//  get system time 
// ============================================================
#include <time.h>


// ============================================================
// time in 24 hours hh:mm:ss format
// ============================================================
static std::string TimeStamp()
{
	char str[9];

	// get the time, and convert it to struct tm format
	time_t a = time(0);
	struct tm* b = localtime(&a);

	// print the time to the string
	strftime(str, 9, "%H:%M:%S", b);

	return str;
}

// ============================================================
// date YYYY:MM:DD format
// ============================================================
static std::string DateStamp()
{
	char str[11];

	// get the time, and convert it to struct tm format
	time_t a = time(0);
	struct tm* b = localtime(&a);

	// print the time to the string
	strftime(str, 11, "%Y.%m.%d", b);

	return str;
}


// ============================================================
// Here is an example of a simple log decorator, you can define your own decorator
// ============================================================
class TextDecorator
{
public:
	static std::string FileHeader(const std::string& p_title)
	{
		return "==================================================\n" +
			p_title + "\n" +
			"==================================================\n\n";
	}

	static std::string SessionOpen()
	{
		return "\n";
	}

	static std::string SessionClose()
	{
		return "\n";
	}

	static std::string Decorate(const std::string& p_string)
	{
		return p_string + "\n";
	}
};


// ============================================================
// New Logger with a new log file and new log title
// ============================================================
template<class decorator>
class Logger
{
public:
	Logger(const std::string& p_filename,
		const std::string& p_logtitle,
		bool p_timestamp = false,
		bool p_datestamp = false);

	~Logger();
	void Log(const std::string& p_entry);

protected:
	std::fstream m_logfile;
	bool m_timestamp;
	bool m_datestamp;

};


typedef Logger<TextDecorator> TextLog;



template<class decorator>
Logger<decorator>::Logger(const std::string& p_filename,
	const std::string& p_logtitle,
	bool p_timestamp,
	bool p_datestamp)
{
	// now the tricky part... testing to see if a file is open or not.
    // stupid C++. You need to open a file in read mode, and if it doesn't
    // open correctly, you know that it doesn't exist.
	// a file is open or not.
	std::fstream filetester(p_filename.c_str(), std::ios::in);

	if (filetester.is_open())
	{
		// the file exists, so just close the test file
		filetester.close();

		// open the real file and set app mode
		m_logfile.open(p_filename.c_str(), std::ios::out | std::ios::app);
	}
	else
	{
		// file doesn't exist.
		m_logfile.open(p_filename.c_str(), std::ios::out);

		// print out a file header to the file
		m_logfile << decorator::FileHeader(p_logtitle);
	}

	// print out an opening statement. Make sure it is time-and-date-stamped
	m_timestamp = true;
	m_datestamp = true;
	m_logfile << decorator::SessionOpen();
	Log("Session opened.");
	m_timestamp = p_timestamp;
	m_datestamp = p_datestamp;

}

template< class decorator >
Logger< decorator >::~Logger()
{
	m_timestamp = true;
	m_datestamp = true;
	Log("Session closed.");
	m_logfile << decorator::SessionClose();

}


template< class decorator >
void Logger< decorator >::Log(const std::string& p_entry)
{
	std::string message;

	if (m_datestamp)
	{
		message += "[" + DateStamp() + "] ";
	}
	if (m_timestamp)
	{
		message += "[" + TimeStamp() + "] ";
	}

	message += p_entry;
	m_logfile << decorator::Decorate(message);
	m_logfile.flush();
}





#endif