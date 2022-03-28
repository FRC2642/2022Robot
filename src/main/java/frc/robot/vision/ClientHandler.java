package frc.robot.vision;

import java.net.*;

import java.util.List;

import java.io.*;

import com.google.gson.*;

import frc.robot.subsystems.BallVisionSubsystem;


public class ClientHandler extends Thread {

    Socket client;

    public ClientHandler(Socket client) {
        this.client = client;
    }
    @SuppressWarnings("unchecked") // hush, sweet child
    private void handleRequest() {
        try {
            System.out.println("Debug: got new client " + client.toString());
            BufferedReader br = new BufferedReader(new InputStreamReader(client.getInputStream()));
            String line;
            String data = "";
            while (!(line = br.readLine()).isEmpty()) {
                if (line.startsWith("data")) {
                    data = line.replace("data: ", "");
                }
            }
            Gson gson = new Gson();
            List<Double> data_out = gson.fromJson(data, List.class);
            BallVisionSubsystem.setCenterX(data_out.get(0));
            OutputStream clientOutput = client.getOutputStream();
            clientOutput.write("HTTP/1.1 200 OK\r\n".getBytes());
            clientOutput.write(("ContentType: application/json\r\n").getBytes());
            clientOutput.write("\r\n".getBytes());
            clientOutput.write("\r\n\r\n".getBytes());
            clientOutput.flush();
            client.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    @Override
    public void run() {
        handleRequest();
    }
}
