classdef Lab2_GUI < matlab.apps.AppBase

    properties (Access = private)
        Lab2  % Instance of Lab2_Assignment class
    end
    
    properties (Constant)
        estopStatus = 'Run'; % Initialise to estopStatus
    end

    methods
        function self = Lab2_GUI 
            % Create an instance of Lab2 Class
            self.Lab2 = Lab2_Assignment();
            self.estopStatus();
        end
    end

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                     matlab.ui.Figure
        TabGroup                     matlab.ui.container.TabGroup
        HomeTab                      matlab.ui.container.Tab
        TextArea_2                   matlab.ui.control.TextArea
        Image11                      matlab.ui.control.Image
        Image10                      matlab.ui.control.Image
        Image9                       matlab.ui.control.Image
        WhatsinstockButton           matlab.ui.control.Button
        Label                        matlab.ui.control.Label
        StartheretogettheShelfElfbottoprepareyoureverydaymealLabel  matlab.ui.control.Label
        GETSTARTEDButton             matlab.ui.control.StateButton
        EstopSwitch                  matlab.ui.control.Switch
        EstopSwitchLabel             matlab.ui.control.Label
        SeeallMealsButton            matlab.ui.control.Button
        MyprofileButton              matlab.ui.control.Button
        Image13                      matlab.ui.control.Image
        OrderTab                     matlab.ui.container.Tab
        HomePageButton               matlab.ui.control.StateButton
        GalleryButton_2              matlab.ui.control.StateButton
        CategoriesPanel_2            matlab.ui.container.Panel
        ConfirmOrderButton           matlab.ui.control.StateButton
        Image18                      matlab.ui.control.Image
        Image17                      matlab.ui.control.Image
        Image16                      matlab.ui.control.Image
        MealsoftheDayListBox_2       matlab.ui.control.ListBox
        MealsoftheDayListBox_2Label  matlab.ui.control.Label
        ProfileInformationPanel      matlab.ui.container.Panel
        Image15                      matlab.ui.control.Image
        Image14                      matlab.ui.control.Image
        ListBox                      matlab.ui.control.ListBox
        OrderPerferencesPanel        matlab.ui.container.Panel
        SelectListBox                matlab.ui.control.ListBox
        SelectListBoxLabel           matlab.ui.control.Label
        GalleryTab                   matlab.ui.container.Tab
        WhatsinStockLabel            matlab.ui.control.Label
        DessertButton                matlab.ui.control.StateButton
        SnacksButton                 matlab.ui.control.StateButton
        DrinksButton                 matlab.ui.control.StateButton
        DinnerButton                 matlab.ui.control.StateButton
        LunchButton                  matlab.ui.control.StateButton
        BreakfastButton              matlab.ui.control.StateButton
        OrderPageButton              matlab.ui.control.StateButton
        SearchPageButton             matlab.ui.control.StateButton
        Image6                       matlab.ui.control.Image
        Image5                       matlab.ui.control.Image
        Image4                       matlab.ui.control.Image
        Image3                       matlab.ui.control.Image
        Image2                       matlab.ui.control.Image
        Image                        matlab.ui.control.Image
        SearchTab                    matlab.ui.container.Tab
        TextArea_3                   matlab.ui.control.TextArea
        Hyperlink3                   matlab.ui.control.Hyperlink
        Image19                      matlab.ui.control.Image
        GalleryButton                matlab.ui.control.StateButton
        TextArea                     matlab.ui.control.TextArea
        Hyperlink2                   matlab.ui.control.Hyperlink
        Image12                      matlab.ui.control.Image
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: MyprofileButton
        function MyprofileButtonPushed(app, event)
            % Switch to the "Order" tab
            app.TabGroup.SelectedTab = app.OrderTab;
        end

        % Button pushed function: WhatsinstockButton
        function WhatsinstockButtonPushed(app, event)
            % Switch to the "Order" tab
            app.TabGroup.SelectedTab = app.GalleryTab;
        end

        % Button pushed function: SeeallMealsButton
        function SeeallMealsButtonPushed(app, event)
            app.TabGroup.SelectedTab = app.OrderTab;
        end

        % Value changed function: GETSTARTEDButton
        function GETSTARTEDButtonValueChanged(app, event)
            app.TabGroup.SelectedTab = app.OrderTab;
        end

        % Value changed function: GalleryButton
        function GalleryButtonValueChanged(app, event)
            value = app.GalleryButton.Value;
            app.TabGroup.SelectedTab = app.GalleryTab;
        end

        % Value changed function: OrderPageButton
        function OrderPageButtonValueChanged(app, event)
            value = app.OrderPageButton.Value;
            app.TabGroup.SelectedTab = app.HomeTab;
        end

        % Value changed function: SearchPageButton
        function SearchPageButtonValueChanged(app, event)
            value = app.SearchPageButton.Value;
            app.TabGroup.SelectedTab = app.SearchTab;
        end

        % Value changed function: GalleryButton_2
        function GalleryButton_2ValueChanged(app, event)
            value = app.GalleryButton_2.Value;
            app.TabGroup.SelectedTab = app.GalleryTab;
        end

        % Value changed function: EstopSwitch
        function EstopSwitchValueChanged(app, event)
            value = app.EstopSwitch.Value;
            
            if strcmp(value, 'Stop')
                Lab2_Assignment.toggleEstop(app.estopStatus); % Set the E-stop status to 'Stop'
            else
                Lab2_Assignment.toggleEstop(app.estopstatus); % Set the E-stop status to 'Run'
            end
            %%
            if strcmp(value, 'Stop')
                estopStatus = 'Stop';
            else
                estopStatus = 'Run';
            end
            Lab2_Assignment.toggleEstop(estopStatus); % Call the toggleEstop method with estopStatus
            
        end

        % Value changed function: HomePageButton
        function HomePageButtonValueChanged(app, event)
            value = app.HomePageButton.Value;
            app.TabGroup.SelectedTab = app.HomeTab;
        end
    
    end
end