#ifndef USERDATA_H_
#define USERDATA_H_

#include <cstdio>

class UserData
{
public:
	inline UserData(const char* label = NULL);
	virtual ~UserData(){}

	const char* getLabel()const{ return mLabel;}
	void setLabel(const char* label){ sprintf(mLabel, "%s", label);}

protected:
	char mLabel[2048];
};

//=== inline methods ==================================================================================

inline UserData::UserData(const char* label)
{
	if ( label )
		sprintf(mLabel, "%s", label);
	else
		sprintf(mLabel, "no label");
}

//=====================================================================================================

#endif /*USERDATA_H_*/
