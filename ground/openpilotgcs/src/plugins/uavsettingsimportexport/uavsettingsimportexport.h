/**
 ******************************************************************************
 *
 * @file       uavsettingsimportexport.h
 * @author     (C) 2011 The OpenPilot Team, http://www.openpilot.org
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup UAVSettingsImportExport UAVSettings Import/Export Plugin
 * @{
 * @brief UAVSettings Import/Export Plugin
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#ifndef UAVSETTINGSIMPORTEXPORT_H 
#define UAVSETTINGSIMPORTEXPORT_H 

#include <extensionsystem/iplugin.h> 
#include "uavobjectutil/uavobjectutilmanager.h"
#include "importsummary.h"
#include "../../../../../build/ground/openpilotgcs/gcsversioninfo.h"
class UAVSettingsImportExportPlugin : public ExtensionSystem::IPlugin 
{ 
    Q_OBJECT

public: 
   UAVSettingsImportExportPlugin(); 
   ~UAVSettingsImportExportPlugin(); 

   void extensionsInitialized(); 
   bool initialize(const QStringList & arguments, QString * errorString); 
   void shutdown(); 

private:
   QString createXMLDocument(const QString docName,
                             const bool isSettings,
                             const bool fullExport);

private slots:
   void importUAVSettings();
   void exportUAVSettings();
   void exportUAVData();

}; 

#endif // UAVSETTINGSIMPORTEXPORT_H
