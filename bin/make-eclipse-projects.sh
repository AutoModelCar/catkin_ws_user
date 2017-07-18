#!/bin/bash

# recommended script call: ./bin/make-eclipse-projects.sh [optional: catkin_project_name]

debug="-DCMAKE_BUILD_TYPE=Debug"				# Buildartifacts will contain debug symbols.
preprocessor_gcc_settings="yes"					# Project settings will contain all gcc include paths and settings (like C++11 enabled).
continue_on_failure="--continue-on-failure"		# Catkin will not stop on failure while configuring each package for eclipse.
source_project=""								# Creates an additional eclipse project inside the source folder (recommended only for eclipse's integrated version control, but not for building)
project_arg=""									# Name of catkin project

# parse option arguments
while getopts "dscpj" opt
do
	case $opt in
		s)
			source_project="-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE"
			echo "using flag: $source_project"
			;;
		*)
			exit
			;;
	esac
done

# last argument can be a catkin_project_name
shift $(($OPTIND - 1))
project_arg=$1

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR
cd ../

# run cmake and build with catkin
catkin build $continue_on_failure --no-status --force-cmake -G"Eclipse CDT4 - Unix Makefiles" $debug $source_project $project_arg

# copy additional eclipse settings into project folders
ROOT=$PWD
cd build

if [ -z $project_arg ]; then
	TMP_PROJECTS_FIND=`find $PWD -name .project`
	PROJECTS=${TMP_PROJECTS_FIND[*]}
else
	TMP_PROJECTS_FIND=`find $PWD -wholename "*/$project_arg/.project"`
	PROJECTS=${TMP_PROJECTS_FIND[*]}
fi

for PROJECT in $PROJECTS; do
	DIR=`dirname $PROJECT`
	echo "Project folder: $DIR"
	cd $DIR
	awk -f $(rospack find mk)/eclipse.awk .project > .project_with_env && mv .project_with_env .project
	if [ "$preprocessor_gcc_settings" == "yes" ]; then
		mkdir -p .settings
		echo '<?xml version="1.0" encoding="UTF-8" standalone="no"?>
		<project>
			<configuration id="org.eclipse.cdt.core.default.config.1" name="Configuration">
				<extension point="org.eclipse.cdt.core.LanguageSettingsProvider">
					<provider-reference id="org.eclipse.cdt.managedbuilder.core.GCCBuiltinSpecsDetector" ref="shared-provider"/>
					<provider copy-of="extension" id="org.eclipse.cdt.ui.UserLanguageSettingsProvider"/>
					<provider-reference id="org.eclipse.cdt.core.ReferencedProjectsLanguageSettingsProvider" ref="shared-provider"/>
					<provider-reference id="org.eclipse.cdt.core.PathEntryScannerInfoLanguageSettingsProvider" ref="shared-provider"/>
				</extension>
			</configuration>
		</project>' > .settings/language.settings.xml
	fi
done
cd $ROOT
