﻿namespace Engine.Forms
{
    partial class GenomeVisualizerForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.panelNetWorkViewer = new System.Windows.Forms.Panel();
            this.listBoxConnections = new System.Windows.Forms.ListBox();
            this.hScrollBarConnectionWeights = new System.Windows.Forms.HScrollBar();
            this.label1 = new System.Windows.Forms.Label();
            this.weightTextBox = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            this.numericUpDownX = new System.Windows.Forms.NumericUpDown();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.numericUpDownY = new System.Windows.Forms.NumericUpDown();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.numericUpDownTolerance = new System.Windows.Forms.NumericUpDown();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownY)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownTolerance)).BeginInit();
            this.SuspendLayout();
            // 
            // panelNetWorkViewer
            // 
            this.panelNetWorkViewer.Location = new System.Drawing.Point(16, 15);
            this.panelNetWorkViewer.Margin = new System.Windows.Forms.Padding(4);
            this.panelNetWorkViewer.Name = "panelNetWorkViewer";
            this.panelNetWorkViewer.Size = new System.Drawing.Size(479, 452);
            this.panelNetWorkViewer.TabIndex = 0;
            // 
            // listBoxConnections
            // 
            this.listBoxConnections.FormattingEnabled = true;
            this.listBoxConnections.ItemHeight = 16;
            this.listBoxConnections.Location = new System.Drawing.Point(515, 15);
            this.listBoxConnections.Margin = new System.Windows.Forms.Padding(4);
            this.listBoxConnections.Name = "listBoxConnections";
            this.listBoxConnections.Size = new System.Drawing.Size(339, 404);
            this.listBoxConnections.TabIndex = 1;
            this.listBoxConnections.SelectedIndexChanged += new System.EventHandler(this.listBoxConnections_SelectedIndexChanged);
            this.listBoxConnections.KeyDown += new System.Windows.Forms.KeyEventHandler(this.listBoxConnections_KeyDown);
            this.listBoxConnections.MouseDown += new System.Windows.Forms.MouseEventHandler(this.listBoxConnections_MouseDown);
            this.listBoxConnections.MouseMove += new System.Windows.Forms.MouseEventHandler(this.listBoxConnections_MouseMove);
            this.listBoxConnections.MouseUp += new System.Windows.Forms.MouseEventHandler(this.listBoxConnections_MouseUp);
            // 
            // hScrollBarConnectionWeights
            // 
            this.hScrollBarConnectionWeights.Location = new System.Drawing.Point(748, 497);
            this.hScrollBarConnectionWeights.Maximum = 1000;
            this.hScrollBarConnectionWeights.Minimum = -1000;
            this.hScrollBarConnectionWeights.Name = "hScrollBarConnectionWeights";
            this.hScrollBarConnectionWeights.Size = new System.Drawing.Size(107, 17);
            this.hScrollBarConnectionWeights.TabIndex = 2;
            this.hScrollBarConnectionWeights.ValueChanged += new System.EventHandler(this.hScrollBarConnectionWeights_ValueChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(515, 438);
            this.label1.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(56, 17);
            this.label1.TabIndex = 4;
            this.label1.Text = "Weight:";
            // 
            // weightTextBox
            // 
            this.weightTextBox.Location = new System.Drawing.Point(577, 438);
            this.weightTextBox.Margin = new System.Windows.Forms.Padding(4);
            this.weightTextBox.Name = "weightTextBox";
            this.weightTextBox.Size = new System.Drawing.Size(121, 22);
            this.weightTextBox.TabIndex = 5;
            this.weightTextBox.KeyDown += new System.Windows.Forms.KeyEventHandler(this.weightTextBox_KeyDown);
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(712, 438);
            this.button1.Margin = new System.Windows.Forms.Padding(4);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(143, 28);
            this.button1.TabIndex = 3;
            this.button1.Text = "Reset";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // numericUpDownX
            // 
            this.numericUpDownX.DecimalPlaces = 3;
            this.numericUpDownX.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericUpDownX.Location = new System.Drawing.Point(905, 15);
            this.numericUpDownX.Margin = new System.Windows.Forms.Padding(4);
            this.numericUpDownX.Maximum = new decimal(new int[] {
            15,
            0,
            0,
            65536});
            this.numericUpDownX.Minimum = new decimal(new int[] {
            15,
            0,
            0,
            -2147418112});
            this.numericUpDownX.Name = "numericUpDownX";
            this.numericUpDownX.Size = new System.Drawing.Size(73, 22);
            this.numericUpDownX.TabIndex = 7;
            this.numericUpDownX.ValueChanged += new System.EventHandler(this.numericUpDownX_ValueChanged);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(883, 16);
            this.label2.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(17, 17);
            this.label2.TabIndex = 8;
            this.label2.Text = "X";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(883, 52);
            this.label3.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(17, 17);
            this.label3.TabIndex = 9;
            this.label3.Text = "Y";
            // 
            // numericUpDownY
            // 
            this.numericUpDownY.DecimalPlaces = 3;
            this.numericUpDownY.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericUpDownY.Location = new System.Drawing.Point(905, 47);
            this.numericUpDownY.Margin = new System.Windows.Forms.Padding(4);
            this.numericUpDownY.Maximum = new decimal(new int[] {
            15,
            0,
            0,
            65536});
            this.numericUpDownY.Minimum = new decimal(new int[] {
            15,
            0,
            0,
            -2147418112});
            this.numericUpDownY.Name = "numericUpDownY";
            this.numericUpDownY.Size = new System.Drawing.Size(73, 22);
            this.numericUpDownY.TabIndex = 10;
            this.numericUpDownY.ValueChanged += new System.EventHandler(this.numericUpDownY_ValueChanged);
            // 
            // pictureBox1
            // 
            this.pictureBox1.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pictureBox1.Location = new System.Drawing.Point(1000, 16);
            this.pictureBox1.Margin = new System.Windows.Forms.Padding(4);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(266, 246);
            this.pictureBox1.TabIndex = 11;
            this.pictureBox1.TabStop = false;
            this.pictureBox1.MouseClick += new System.Windows.Forms.MouseEventHandler(this.pictureBox1_MouseClick);
            this.pictureBox1.MouseMove += new System.Windows.Forms.MouseEventHandler(this.pictureBox1_MouseMove);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(1005, 277);
            this.label4.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(0, 17);
            this.label4.TabIndex = 12;
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(883, 92);
            this.label5.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(72, 17);
            this.label5.TabIndex = 13;
            this.label5.Text = "Tolerance";
            // 
            // numericUpDownTolerance
            // 
            this.numericUpDownTolerance.DecimalPlaces = 2;
            this.numericUpDownTolerance.Increment = new decimal(new int[] {
            5,
            0,
            0,
            131072});
            this.numericUpDownTolerance.Location = new System.Drawing.Point(887, 112);
            this.numericUpDownTolerance.Margin = new System.Windows.Forms.Padding(4);
            this.numericUpDownTolerance.Maximum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.numericUpDownTolerance.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            -2147483648});
            this.numericUpDownTolerance.Name = "numericUpDownTolerance";
            this.numericUpDownTolerance.Size = new System.Drawing.Size(92, 22);
            this.numericUpDownTolerance.TabIndex = 14;
            this.numericUpDownTolerance.Value = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericUpDownTolerance.ValueChanged += new System.EventHandler(this.numericUpDownTolerance_ValueChanged);
            // 
            // GenomeVisualizerForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1290, 479);
            this.Controls.Add(this.numericUpDownTolerance);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.pictureBox1);
            this.Controls.Add(this.numericUpDownY);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.numericUpDownX);
            this.Controls.Add(this.weightTextBox);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.hScrollBarConnectionWeights);
            this.Controls.Add(this.listBoxConnections);
            this.Controls.Add(this.panelNetWorkViewer);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedToolWindow;
            this.Margin = new System.Windows.Forms.Padding(4);
            this.Name = "GenomeVisualizerForm";
            this.Text = "Genome CPPN Visualizer";
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownY)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownTolerance)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Panel panelNetWorkViewer;
        private System.Windows.Forms.ListBox listBoxConnections;
        private System.Windows.Forms.HScrollBar hScrollBarConnectionWeights;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox weightTextBox;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.NumericUpDown numericUpDownX;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.NumericUpDown numericUpDownY;
        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.NumericUpDown numericUpDownTolerance;
    }
}