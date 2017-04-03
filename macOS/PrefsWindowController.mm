/*
 *  PrefsWindowController.mm
 *  ARToolKit6
 *
 *  This file is part of ARToolKit.
 *
 *  ARToolKit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ARToolKit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with ARToolKit.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2017-2017 Daqri LLC. All Rights Reserved.
 *
 *  Author(s): Philip Lamb
 *
 */


#import "../prefs.hpp"
#import "PrefsWindowController.h"
#import <AR6/ARVideo/video.h>
#import "../calib_camera.h"

static NSString *const kSettingCalibrationPatternType = @"calibrationPatternType";
static NSString *const kSettingCalibrationPatternSizeWidth = @"calibrationPatternSizeWidth";
static NSString *const kSettingCalibrationPatternSizeHeight = @"calibrationPatternSizeHeight";
static NSString *const kSettingCalibrationPatternSpacing = @"calibrationPatternSpacing";
static NSString *const kSettingCalibrationServerUploadURL = @"calibrationServerUploadURL";
static NSString *const kSettingCalibrationServerAuthenticationToken = @"calibrationServerAuthenticationToken";

static NSString *const kCalibrationPatternTypeChessboardStr = @"Chessboard";
static NSString *const kCalibrationPatternTypeCirclesStr = @"Circles";
static NSString *const kCalibrationPatternTypeAsymmetricCirclesStr = @"Asymmetric circles";

@interface PrefsWindowController ()
{
    IBOutlet NSButton *showPrefsOnStartup;
    IBOutlet NSTextField *calibrationServerUploadURL;
    IBOutlet NSTextField *calibrationServerAuthenticationToken;
    IBOutlet NSPopUpButton *cameraInputPopup;
    IBOutlet NSPopUpButton *cameraPresetPopup;
    __weak IBOutlet NSStepper *calibrationPatternSizeWidthStepper;
    __weak IBOutlet NSTextField *calibrationPatternSizeWidthLabel;
    __weak IBOutlet NSStepper *calibrationPatternSizeHeightStepper;
    __weak IBOutlet NSTextField *calibrationPatternSizeHeightLabel;
    __weak IBOutlet NSTextField *calibrationPatternSpacing;
    __weak IBOutlet NSSegmentedControl *calibrationPatternTypeControl;
}
- (IBAction)calibrationPatternTypeChanged:(NSSegmentedControl *)sender;
- (IBAction)okSelected:(NSButton *)sender;
@end

@implementation PrefsWindowController

- (instancetype)initWithWindowNibName:(NSString *)windowNibName
{
    return [super initWithWindowNibName:windowNibName];
}

- (instancetype)initWithWindow:(NSWindow *)window
{
    id ret;
    if ((ret = [super initWithWindow:window])) {
        // Customisation here.
    }
    return (ret);
}

- (void)windowDidLoad {
    [super windowDidLoad];
    
    // Pre-process, selecting options etc.
    NSUserDefaults* defaults = [NSUserDefaults standardUserDefaults];
    
    // Populate the camera input popup.
    ARVideoSourceInfoListT *sil = ar2VideoCreateSourceInfoList("-module=AVFoundation");
    if (!sil) {
        ARLOGe("Unable to get ARVideoSourceInfoListT.\n");
        cameraInputPopup.enabled = FALSE;
    } else if (sil->count == 0) {
        ARLOGe("No video sources connected.\n");
        cameraInputPopup.enabled = FALSE;
    } else {
        NSString *cot = [defaults stringForKey:@"cameraOpenToken"];
        int selectedItemIndex = 0;
        for (int i = 0; i < sil->count; i++) {
            [cameraInputPopup addItemWithTitle:@(sil->info[i].name)];
            [cameraInputPopup itemAtIndex:i].representedObject = @(sil->info[i].open_token);
            if (cot && sil->info[i].open_token && strcmp(cot.UTF8String, sil->info[i].open_token) == 0) {
                selectedItemIndex = i;
            }
        }
        [cameraInputPopup selectItemAtIndex:selectedItemIndex];
        cameraInputPopup.enabled = TRUE;
    }
    
    NSString *cp = [defaults stringForKey:@"cameraPreset"];
    if (cp) [cameraPresetPopup selectItemWithTitle:cp];
    
    NSString *csuu = [defaults stringForKey:kSettingCalibrationServerUploadURL];
    calibrationServerUploadURL.stringValue = (csuu ? csuu : @"");
    calibrationServerUploadURL.placeholderString = @CALIBRATION_SERVER_UPLOAD_URL_DEFAULT;
    NSString *csat = [defaults stringForKey:kSettingCalibrationServerAuthenticationToken];
    calibrationServerAuthenticationToken.stringValue = (csat ? csat : @"");
    calibrationServerAuthenticationToken.placeholderString = @CALIBRATION_SERVER_AUTHENTICATION_TOKEN_DEFAULT;
    
    showPrefsOnStartup.state = [defaults boolForKey:@"showPrefsOnStartup"];
    
    Calibration::CalibrationPatternType patternType = CALIBRATION_PATTERN_TYPE_DEFAULT;
    NSString *patternTypeStr = [defaults objectForKey:kSettingCalibrationPatternType];
    if (patternTypeStr.length != 0) {
        if ([patternTypeStr isEqualToString:kCalibrationPatternTypeChessboardStr]) patternType = Calibration::CalibrationPatternType::CHESSBOARD;
        else if ([patternTypeStr isEqualToString:kCalibrationPatternTypeCirclesStr]) patternType = Calibration::CalibrationPatternType::CIRCLES_GRID;
        else if ([patternTypeStr isEqualToString:kCalibrationPatternTypeAsymmetricCirclesStr]) patternType = Calibration::CalibrationPatternType::ASYMMETRIC_CIRCLES_GRID;
    }
    switch (patternType) {
        case Calibration::CalibrationPatternType::CHESSBOARD: calibrationPatternTypeControl.selectedSegment = 0; break;
        case Calibration::CalibrationPatternType::CIRCLES_GRID: calibrationPatternTypeControl.selectedSegment = 1; break;
        case Calibration::CalibrationPatternType::ASYMMETRIC_CIRCLES_GRID: calibrationPatternTypeControl.selectedSegment = 2; break;
    }
    
    int w = (int)[defaults integerForKey:kSettingCalibrationPatternSizeWidth];
    int h = (int)[defaults integerForKey:kSettingCalibrationPatternSizeHeight];
    if (w < 1 || h < 1) {
        w = Calibration::CalibrationPatternSizes[patternType].width;
        h = Calibration::CalibrationPatternSizes[patternType].height;
    }
    calibrationPatternSizeWidthStepper.intValue = w;
    calibrationPatternSizeHeightStepper.intValue = h;
    [calibrationPatternSizeWidthLabel takeIntValueFrom:calibrationPatternSizeWidthStepper];
    [calibrationPatternSizeHeightLabel takeIntValueFrom:calibrationPatternSizeHeightStepper];
    
    float f = [defaults floatForKey:kSettingCalibrationPatternSpacing];
    if (f <= 0.0f) f = Calibration::CalibrationPatternSpacings[patternType];
    calibrationPatternSpacing.stringValue = [NSString stringWithFormat:@"%.2f", f];
}

- (IBAction)calibrationPatternTypeChanged:(NSSegmentedControl *)sender
{
    Calibration::CalibrationPatternType patternType;
    switch (sender.selectedSegment) {
        case 0:
            patternType = Calibration::CalibrationPatternType::CHESSBOARD;
            break;
        case 1:
            patternType = Calibration::CalibrationPatternType::CIRCLES_GRID;
            break;
        case 2:
            patternType = Calibration::CalibrationPatternType::ASYMMETRIC_CIRCLES_GRID;
            break;
        default:
            patternType = CALIBRATION_PATTERN_TYPE_DEFAULT;
            break;
    }
    calibrationPatternSizeWidthStepper.intValue = Calibration::CalibrationPatternSizes[patternType].width;
    calibrationPatternSizeHeightStepper.intValue = Calibration::CalibrationPatternSizes[patternType].height;
    [calibrationPatternSizeWidthLabel takeIntValueFrom:calibrationPatternSizeWidthStepper];
    [calibrationPatternSizeHeightLabel takeIntValueFrom:calibrationPatternSizeHeightStepper];
    calibrationPatternSpacing.stringValue = [NSString stringWithFormat:@"%.2f", Calibration::CalibrationPatternSpacings[patternType]];
}

- (BOOL)windowShouldClose:(id)sender
{
    return (YES);
}

- (void)windowWillClose:(NSNotification *)notification
{
    // Post-process selected options.
}

- (IBAction)okSelected:(NSButton *)sender {
    
    NSUserDefaults* defaults = [NSUserDefaults standardUserDefaults];
    NSString *cot = cameraInputPopup.selectedItem.representedObject;
    [defaults setObject:cameraPresetPopup.selectedItem.title forKey:@"cameraPreset"];
    [defaults setObject:cot forKey:@"cameraOpenToken"];
    [defaults setObject:calibrationServerUploadURL.stringValue forKey:kSettingCalibrationServerUploadURL];
    [defaults setObject:calibrationServerAuthenticationToken.stringValue forKey:kSettingCalibrationServerAuthenticationToken];
    [defaults setBool:showPrefsOnStartup.state forKey:@"showPrefsOnStartup"];
    switch (calibrationPatternTypeControl.selectedSegment) {
        case 0:
            [defaults setObject:kCalibrationPatternTypeChessboardStr forKey:kSettingCalibrationPatternType];
            break;
        case 1:
            [defaults setObject:kCalibrationPatternTypeCirclesStr forKey:kSettingCalibrationPatternType];
            break;
        case 2:
            [defaults setObject:kCalibrationPatternTypeAsymmetricCirclesStr forKey:kSettingCalibrationPatternType];
            break;
        default:
            [defaults setObject:nil forKey:kSettingCalibrationPatternType];
            break;
    }
    [defaults setInteger:calibrationPatternSizeWidthStepper.intValue forKey:kSettingCalibrationPatternSizeWidth];
    [defaults setInteger:calibrationPatternSizeHeightStepper.intValue forKey:kSettingCalibrationPatternSizeHeight];
    [defaults setFloat:calibrationPatternSpacing.floatValue forKey:kSettingCalibrationPatternSpacing];
    
    [NSApp stopModal];
    [self close];
    
    SDL_Event event;
    SDL_zero(event);
    event.type = gSDLEventPreferencesChanged;
    event.user.code = (Sint32)0;
    event.user.data1 = NULL;
    event.user.data2 = NULL;
    SDL_PushEvent(&event);
}

-(void) showHelp:(id)sender
{
    [[NSWorkspace sharedWorkspace] openURL:[NSURL URLWithString:@"https://github.com/artoolkit/artoolkit6/wiki/Camera-calibration-macOS"]];
}

@end

//
// C interface to our ObjC preferences class.
//

void *initPreferences(void)
{
    // Register the preference defaults early.
    NSDictionary *appDefaults = @{@"showPrefsOnStartup": @YES};
    [[NSUserDefaults standardUserDefaults] registerDefaults:appDefaults];

    //NSLog(@"showPrefsOnStartup=%s.\n", ([[NSUserDefaults standardUserDefaults] boolForKey:@"showPrefsOnStartup"] ? "true" : "false"));
    
    PrefsWindowController *pwc = [[PrefsWindowController alloc] initWithWindowNibName:@"PrefsWindow"];
    
    // Register the Preferences menu item in the app menu.
    NSMenu *appMenu = [NSApp.mainMenu itemAtIndex:0].submenu;
    for (NSMenuItem *mi in appMenu.itemArray) {
        if ([mi.title isEqualToString:@"Preferences…"]) {
            mi.target = pwc;
            mi.action = @selector(showWindow:);
            mi.enabled = TRUE;
            break;
        }
    }
    
    // Add the Help menu and an item for the app.
    NSMenu *helpMenu = [[NSMenu alloc] initWithTitle:@"Help"];
    NSMenuItem *helpMenu0 = [[NSMenuItem alloc] init];
    helpMenu0.submenu = helpMenu;
    NSString *appName = [[NSBundle mainBundle] objectForInfoDictionaryKey:@"CFBundleName"];
    NSMenuItem *helpMenuItem = [[NSMenuItem alloc] initWithTitle:[appName stringByAppendingString:@" Help"] action:@selector(showHelp:) keyEquivalent:@"?"];
    helpMenuItem.target = pwc;
    [helpMenu addItem:helpMenuItem];
    [NSApp.mainMenu addItem:helpMenu0];
    NSApp.helpMenu = helpMenu;
    
    if ([[NSUserDefaults standardUserDefaults] boolForKey:@"showPrefsOnStartup"]) {
        showPreferences((__bridge void *)pwc);
    }
    return ((void *)CFBridgingRetain(pwc));
}

void showPreferences(void *preferences)
{
    PrefsWindowController *pwc = (__bridge PrefsWindowController *)preferences;
    if (pwc) {
        [pwc showWindow:pwc];
        [pwc.window makeKeyAndOrderFront:pwc];
        //[NSApp runModalForWindow:pwc.window];
        //NSLog(@"Back from modal\n");
    }
}

char *getPreferenceCameraOpenToken(void *preferences)
{
    NSString *cot = [[NSUserDefaults standardUserDefaults] stringForKey:@"cameraOpenToken"];
    if (cot.length != 0) return (strdup(cot.UTF8String));
    return NULL;
}

char *getPreferenceCameraResolutionToken(void *preferences)
{
    NSString *cp = [[NSUserDefaults standardUserDefaults] stringForKey:@"cameraPreset"];
    if (cp.length != 0) {
        return (strdup([NSString stringWithFormat:@"-preset=%@", cp].UTF8String));
    }
    return NULL;
}

char *getPreferenceCalibrationServerUploadURL(void *preferences)
{
    NSString *csuu = [[NSUserDefaults standardUserDefaults] stringForKey:kSettingCalibrationServerUploadURL];
    if (csuu.length != 0) return (strdup(csuu.UTF8String));
    return (strdup(CALIBRATION_SERVER_UPLOAD_URL_DEFAULT));
}

char *getPreferenceCalibrationServerAuthenticationToken(void *preferences)
{
    NSString *csat = [[NSUserDefaults standardUserDefaults] stringForKey:kSettingCalibrationServerAuthenticationToken];
    if (csat.length != 0) return (strdup(csat.UTF8String));
    return (strdup(CALIBRATION_SERVER_AUTHENTICATION_TOKEN_DEFAULT));
}

Calibration::CalibrationPatternType getPreferencesCalibrationPatternType(void *preferences)
{
    Calibration::CalibrationPatternType patternType = CALIBRATION_PATTERN_TYPE_DEFAULT;
    NSString *patternTypeStr = [[NSUserDefaults standardUserDefaults] objectForKey:kSettingCalibrationPatternType];
    if (patternTypeStr.length != 0) {
        if ([patternTypeStr isEqualToString:kCalibrationPatternTypeChessboardStr]) patternType = Calibration::CalibrationPatternType::CHESSBOARD;
        else if ([patternTypeStr isEqualToString:kCalibrationPatternTypeCirclesStr]) patternType = Calibration::CalibrationPatternType::CIRCLES_GRID;
        else if ([patternTypeStr isEqualToString:kCalibrationPatternTypeAsymmetricCirclesStr]) patternType = Calibration::CalibrationPatternType::ASYMMETRIC_CIRCLES_GRID;
    }
    return patternType;
}

cv::Size getPreferencesCalibrationPatternSize(void *preferences)
{
    int w = (int)[[NSUserDefaults standardUserDefaults] integerForKey:kSettingCalibrationPatternSizeWidth];
    int h = (int)[[NSUserDefaults standardUserDefaults] integerForKey:kSettingCalibrationPatternSizeHeight];
    if (w > 0 && h > 0) return cv::Size(w, h);
    
    return Calibration::CalibrationPatternSizes[getPreferencesCalibrationPatternType(preferences)];
}

float getPreferencesCalibrationPatternSpacing(void *preferences)
{
    float f = [[NSUserDefaults standardUserDefaults] floatForKey:kSettingCalibrationPatternSpacing];
    if (f > 0.0f) return f;
    
    return Calibration::CalibrationPatternSpacings[getPreferencesCalibrationPatternType(preferences)];
}

void preferencesFinal(void **preferences_p)
{
    if (preferences_p) {
        CFRelease(*preferences_p);
        *preferences_p = NULL;
    }
}
